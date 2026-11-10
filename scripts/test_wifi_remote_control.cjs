const {test} = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const path = require('node:path');

function page() {
  const source = fs.readFileSync(path.join(__dirname, '../components/wifi_remote_control/wifi_remote_control.cc'), 'utf8');
  const script = source.match(/<script>([\s\S]*?)<\/script>/)[1];
  const elements = new Map(), sockets = [], intervals = [], timers = [], revoked = [];
  class WebSocket {
    static OPEN = 1;
    constructor(url) {this.url=url; this.readyState=0; this.bufferedAmount=0; this.sent=[]; sockets.push(this);}
    send(data) {this.sent.push(data);}
    close() {this.readyState=3; this.onclose?.();}
    open() {this.readyState=1; this.onopen?.();}
  }
  let nextImage = 0;
  const events = {};
  const document = {hidden:false, addEventListener(name, fn){events[name]=fn;}, getElementById(id) {
    if (!elements.has(id)) elements.set(id, {style:{}, clientWidth:180, handlers:{},
      addEventListener(name, fn){this.handlers[name]=fn;}, setPointerCapture(){},
      getBoundingClientRect(){return {left:0, top:0, width:180, height:180};}});
    return elements.get(id);
  }};
  const context = vm.createContext({document, window:{addEventListener(name,fn){events[name]=fn;}},
    location:{host:'192.168.4.1', hostname:'192.168.4.1', protocol:'http:'}, WebSocket, Blob,
    URL:{createObjectURL(){return 'blob:'+ ++nextImage;},revokeObjectURL(url){revoked.push(url);}},
    setTimeout(fn){timers.push(fn);}, setInterval(fn,ms){intervals.push({fn,ms});},
    performance:{now:()=>1000}, fetch:()=>Promise.resolve(),
  });
  vm.runInContext(script, context);
  return {context, elements, sockets, intervals, timers, revoked, events, document};
}

test('connects to the official /ws endpoint and sends text coordinates', () => {
  const p=page(); assert.equal(p.sockets.length,1);
  const ws=p.sockets[0]; assert.equal(ws.url,'ws://192.168.4.1/ws'); ws.open();
  vm.runInContext('send(0.25,-0.5)',p.context);
  assert.equal(ws.sent.at(-1),'x0.25y-0.50');
});

test('holding joystick still refreshes movement; release stops refresh', () => {
  const p=page(); const ws=p.sockets[0]; assert.ok(ws); ws.open();
  const pad=p.elements.get('pad'); pad.handlers.pointerdown({pointerId:1,clientX:90,clientY:8});
  const tick=p.intervals.find(t=>t.ms===150); assert.ok(tick);
  tick.fn(); tick.fn(); assert.equal(ws.sent.at(-1),'x0.00y1.00');
  pad.handlers.pointerup({pointerId:1}); assert.equal(ws.sent.at(-1),'x0y0');
  const count=ws.sent.length; tick.fn(); assert.equal(ws.sent.length,count);
});

test('congested connection drops movement instead of queuing stale commands', () => {
  const p=page(); const ws=p.sockets[0]; assert.ok(ws); ws.open(); ws.sent=[]; ws.bufferedAmount=100;
  vm.runInContext('send(1,0);send(0,1)',p.context); assert.equal(ws.sent.length,0);
  vm.runInContext('sendStop()',p.context); assert.equal(ws.sent.at(-1),'x0y0');
});

test('binary JPEG updates image and releases previous object URL', () => {
  const p=page(); const ws=p.sockets[0]; assert.ok(ws); ws.open();
  ws.onmessage({data:new Blob(['jpeg1'])}); const cam=p.elements.get('cam'); const first=cam.src;
  assert.equal(first,'blob:1'); cam.onload();
  ws.onmessage({data:new Blob(['jpeg2'])}); assert.equal(cam.src,'blob:2');
  assert.deepEqual(p.revoked,[first]);
});

test('heartbeat uses official PING and disconnect reconnects without old movement', () => {
  const p=page(); const ws=p.sockets[0]; assert.ok(ws); ws.open();
  const heartbeat=p.intervals.find(t=>t.ms===1000); assert.ok(heartbeat); heartbeat.fn();
  assert.equal(ws.sent.at(-1),'PING');
  vm.runInContext('dragging=true',p.context); ws.close();
  assert.equal(p.context.dragging,false); assert.equal(p.timers.length,1);
  p.timers[0](); const next=p.sockets[1]; next.open(); assert.equal(next.sent.at(-1),'x0y0');
});

test('hiding page stops motion', () => {
  const p=page(); const ws=p.sockets[0]; assert.ok(ws); ws.open();
  vm.runInContext('dragging=true',p.context); p.document.hidden=true;
  p.events.visibilitychange(); assert.equal(p.context.dragging,false); assert.equal(ws.sent.at(-1),'x0y0');
});
