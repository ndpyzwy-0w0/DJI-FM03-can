// Browser smoke test for this project's UI. All /api requests are mocked;
// only HTML/CSS/JS are loaded from the idle local server. No serial is opened.
const { chromium } = require('playwright');
const fs = require('node:fs');
const path = require('node:path');

(async () => {
  const url = process.env.FM03_UI_TEST_URL || 'http://127.0.0.1:8765';
  const state = await (await fetch(url+'/api/state')).json();
  if (state.product !== 'fm03_capture') throw Error('Unexpected server');
  Object.assign(state,{phase:'capturing',token:'mock-only',elapsed:20,status_age:0.1,frames:1000,
    status:{bitrate:1000000},motor:{epoch:1,stage:3,coordinate:3763,age_s:0.01,
      fresh:true,can_record:true,observed_range_percent:3763/4096*100,boundary:null,
      marks:{},records:[],history:[],limit_candidate:false}});
  const browser = await chromium.launch({headless:true,
    executablePath:process.env.FM03_BROWSER_PATH || 'C:/Program Files (x86)/Microsoft/Edge/Application/msedge.exe'});
  try {
    const page = await browser.newPage({viewport:{width:1280,height:1000}});
    const errors=[];
    page.on('pageerror',error=>errors.push(error.message));
    await page.route('**/api/**',async route=>{
      const endpoint=new URL(route.request().url()).pathname;
      let body=state;
      if (endpoint==='/api/ports') body=[{port:'MOCK',description:'offline UI fixture',usb:false}];
      else if (endpoint==='/api/coordinate/record') {
        const marker=route.request().postDataJSON().marker;
        state.motor.marks[marker]={marker,coordinate:state.motor.coordinate,epoch:1,valid:true};
      } else if (endpoint!=='/api/state') throw Error('Unexpected mutation: '+endpoint);
      await route.fulfill({json:body});
    });
    await page.goto(url);
    await page.waitForFunction(()=>document.getElementById('motorCoordinate').textContent==='3763');
    await page.locator('#recordA').click();
    await page.waitForFunction(()=>document.getElementById('markA').textContent.includes('3763'));
    state.motor.coordinate=2562;
    state.motor.observed_range_percent=2562/4096*100;
    await page.waitForFunction(()=>document.getElementById('motorCoordinate').textContent==='2562');
    await page.locator('#recordB').click();
    await page.waitForFunction(()=>document.getElementById('markDistance').textContent.includes('-1201'));
    await page.waitForFunction(()=>!document.getElementById('recordA').disabled);
    const directory=path.resolve(__dirname,'../artifacts');
    fs.mkdirSync(directory,{recursive:true});
    await page.screenshot({path:path.join(directory,'status-coordinate-desktop.png'),fullPage:true});
    state.motor.coordinate=5000;
    state.motor.observed_range_percent=null;
    await page.waitForFunction(()=>document.getElementById('motorBoundary').textContent.includes('超出本次观察范围'));
    state.motor.fresh=state.motor.can_record=false;
    state.motor.coordinate=null;
    state.motor.age_s=1;
    for (const mark of Object.values(state.motor.marks)) Object.assign(mark,{valid:false,invalid_reason:'坐标数据过期'});
    await page.waitForFunction(()=>document.getElementById('motorCoordinate').textContent==='—' && document.getElementById('recordA').disabled);
    await page.setViewportSize({width:360,height:900});
    await page.screenshot({path:path.join(directory,'status-coordinate-mobile.png'),fullPage:true});
    const overflow=await page.evaluate(()=>document.documentElement.scrollWidth>innerWidth);
    if (overflow || errors.length) throw Error(JSON.stringify({overflow,errors}));
    console.log('Status UI: A/B record, raw delta, stale disabling, 1280/360px layout and no page errors passed (mock API only).');
  } finally {await browser.close();}
})().catch(error=>{console.error(error);process.exitCode=1;});
