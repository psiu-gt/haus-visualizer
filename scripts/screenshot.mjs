// Dev helper: load the site in headless Chrome, step through modes, save screenshots.
//   node scripts/screenshot.mjs OUT_DIR [url] [--mobile]
import puppeteer from 'puppeteer-core';

const out = process.argv[2] || '.';
const url = process.argv.find((a) => a.startsWith('http')) || 'http://localhost:8000/';
const mobile = process.argv.includes('--mobile');
const chrome = process.env.CHROME || 'C:/Program Files/Google/Chrome/Application/chrome.exe';

const browser = await puppeteer.launch({
  executablePath: chrome,
  headless: 'new',
  args: ['--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'],
});
const page = await browser.newPage();
await page.setViewport(mobile ? { width: 390, height: 844, deviceScaleFactor: 2, isMobile: true, hasTouch: true } : { width: 1400, height: 860 });
page.on('console', (m) => console.log(`[${m.type()}] ${m.text()}`));
page.on('pageerror', (e) => console.log('[pageerror]', e.message));
page.on('requestfailed', (r) => console.log('[requestfailed]', r.url(), r.failure()?.errorText));

await page.goto(url);
await page.waitForFunction(() => document.getElementById('loading').classList.contains('done') || /Couldn't/.test(document.getElementById('loadText').textContent), { timeout: 180000 });
console.log('loading text:', await page.$eval('#loadText', (e) => e.textContent));
const wait = (ms) => new Promise((r) => setTimeout(r, ms));
const shot = async (name) => { await wait(1500); await page.screenshot({ path: `${out}/${name}.png` }); console.log('shot', name); };
const key = (k) => page.keyboard.press(k);

const prefix = mobile ? 'm_' : '';
await shot(prefix + '1_scan_aerial');
await key('Space'); await shot(prefix + '2_model_aerial');
if (mobile) { await page.tap('#roofBtn'); await shot('m_3_noroof'); await browser.close(); process.exit(0); }
await page.$eval('#mix', (e) => { e.value = 0.5; e.dispatchEvent(new Event('input')); }); await shot('3_blend');
await key('KeyS'); await shot('4_swipe');
await key('KeyS'); await key('Digit3'); await wait(800); await shot('5_street_blend');
await key('Digit1'); await key('Space'); await wait(800);
await page.click('#wallsBtn'); await shot('6_walls');
await page.click('#roofBtn'); await page.mouse.move(10, 400); await shot('7_walls_noroof');
await browser.close();
