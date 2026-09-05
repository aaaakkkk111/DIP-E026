const fs = require('fs');
const path = require('path');
const sharp = require('C:/Users/stato/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/sharp');

const colors = {
  blue: ['#EEF5FF', '#5288D4'],
  purple: ['#F3EDFC', '#9363CD'],
  green: ['#EEF7EB', '#779D66'],
  orange: ['#FFF3E5', '#E6A15B'],
};
const esc = s => s.replace(/&/g, '&amp;').replace(/</g, '&lt;');
function text(x, y, value, size = 25, weight = 600, anchor = 'middle', fill = '#142746') {
  return `<text x="${x}" y="${y}" font-size="${size}" font-weight="${weight}" text-anchor="${anchor}" fill="${fill}">${esc(value)}</text>`;
}
function box(x, y, w, h, title, subtitle, color = 'blue', titleSize = 27) {
  const [fill, stroke] = colors[color];
  return `<g><rect x="${x}" y="${y}" width="${w}" height="${h}" rx="14" fill="${fill}" stroke="${stroke}" stroke-width="2"/>${text(x + w/2, y + h/2 - 3, title, titleSize)}${text(x + w/2, y + h/2 + 28, subtitle, 20, 400, 'middle', '#405573')}</g>`;
}
function arrow(d, slow = false, color = '#20334E') {
  return `<path d="${d}" fill="none" stroke="${color}" stroke-width="2.8" stroke-linejoin="round" stroke-linecap="round" ${slow ? 'stroke-dasharray="8 7"' : ''} marker-end="url(#arrow)"/>`;
}
function section(y, h, label, tone = 'blue') {
  const [fill, stroke] = colors[tone];
  return `<rect x="64" y="${y}" width="1392" height="${h}" rx="20" fill="${fill}" fill-opacity="0.28" stroke="${stroke}" stroke-opacity="0.55" stroke-width="1.5"/>${text(92, y + 38, label, 20, 700, 'start', '#526783')}`;
}
function legend(y) {
  return arrow(`M 94 ${y} H 155`) + text(174, y + 7, 'Real-time control', 20, 400, 'start') +
    arrow(`M 415 ${y} H 476`, true) + text(495, y + 7, 'Tuning / training / deployment', 20, 400, 'start') +
    [['purple', 'LLM'], ['orange', 'Review / checks'], ['green', 'Data']].map(([c,l], i) => {
      const x = [890, 1062, 1315][i];
      return `<rect x="${x}" y="${y-13}" width="25" height="25" rx="5" fill="${colors[c][0]}" stroke="${colors[c][1]}"/>` + text(x + 36, y + 7, l, 19, 400, 'start');
    }).join('');
}
function svg(height, title, subtitle, body, desc) {
  return `<svg xmlns="http://www.w3.org/2000/svg" width="1520" height="${height}" viewBox="0 0 1520 ${height}" role="img" aria-labelledby="title desc"><title id="title">${esc(title)}</title><desc id="desc">${esc(desc)}</desc><defs><marker id="arrow" viewBox="0 0 12 12" refX="10" refY="6" markerWidth="9" markerHeight="9" orient="auto-start-reverse"><path d="M 1 1 L 11 6 L 1 11 z" fill="#20334E"/></marker></defs><rect width="1520" height="${height}" fill="white"/><g font-family="Arial, Helvetica, sans-serif">${text(760, 73, title, 43, 700)}${text(760, 117, subtitle, 23, 400, 'middle', '#526783')}${legend(167)}${body}</g></svg>`;
}

let a = section(216, 242, 'STM32  /  REAL-TIME CONTROL');
a += section(520, 245, 'PC  /  AUTOMATIC TUNING', 'purple');
a += arrow('M 390 333 H 625') + arrow('M 895 333 H 1130');
a += text(507, 317, 'State', 20, 400) + text(1010, 317, 'Control output', 20, 400);
a += arrow('M 255 387 V 485 H 48 V 649 H 120', true) + text(274, 492, 'Bluetooth telemetry', 20, 400, 'start');
a += arrow('M 1265 596 V 420 H 760 V 387', true) + text(1009, 407, 'Validated parameters', 20, 400);
a += arrow('M 390 649 H 625', true) + arrow('M 895 649 H 1130', true);
a += text(507, 630, 'Test feedback', 20, 400) + text(1010, 630, 'Candidate settings', 20, 400);
a += box(120, 279, 270, 108, 'Sensors', 'Robot state');
a += box(625, 279, 270, 108, 'PID / LQR', 'Local controller');
a += box(1130, 279, 270, 108, 'Motors + robot', 'Physical response');
a += box(120, 596, 270, 108, 'Logs + metrics', 'Evaluate each test', 'green');
a += box(625, 596, 270, 108, 'LLM tuner', 'Propose parameter changes', 'purple');
a += box(1130, 596, 270, 108, 'Validate + apply', 'Accept or roll back', 'orange', 26);
a += `<rect x="64" y="792" width="1392" height="58" rx="13" fill="#EEF5FF" stroke="#D1E2F8"/>`;
a += text(760, 829, 'LLM output: candidate settings   |   Deployed output: validated PID gains or LQR feedback gains', 23, 600);

let b = section(216, 293, 'PC  /  HUMAN-MEDIATED DESIGN + AUTOMATIC RL TRAINING', 'purple');
b += section(602, 350, 'STM32  /  REAL-TIME RESIDUAL CONTROL');
b += arrow('M 366 351 H 441', true) + arrow('M 709 351 H 784', true) + arrow('M 1052 351 H 1127', true);
b += arrow('M 1264 407 V 470 H 232 V 407', true) + text(754, 454, 'Training results and evaluation metrics', 21, 400);
b += arrow('M 120 760 H 86 V 351 H 98', true) + text(225, 556, 'Bluetooth logs', 21, 400);
b += arrow('M 1398 351 H 1488 V 907 H 634 V 864', true) + text(1230, 556, 'Validated policy deployment', 21, 400);
b += arrow('M 350 760 H 419 V 694 H 500') + arrow('M 419 760 V 818 H 500');
b += `<circle cx="419" cy="760" r="4.5" fill="#20334E"/>`;
b += arrow('M 768 694 H 900 V 742 H 951') + arrow('M 768 818 H 900 V 781 H 951');
b += text(830, 679, 'Base output', 18, 400) + text(830, 847, 'Correction', 18, 400);
b += arrow('M 1115 760 H 1182');
b += box(98, 295, 268, 112, 'Logs + metrics', 'Simulation and robot data', 'green', 26);
b += box(441, 295, 268, 112, 'Team + LLM', 'Design reward changes', 'purple');
b += box(784, 295, 268, 112, 'Team review', 'Approve and start training', 'orange', 26);
b += box(1127, 295, 271, 112, 'PPO train + test', 'Learn and evaluate policy', 'blue', 26);
b += box(120, 710, 230, 100, 'Sensors', 'Robot state');
b += box(500, 648, 268, 92, 'Fixed PID', 'Frozen baseline', 'blue', 26);
b += box(500, 772, 268, 92, 'Neural policy', 'Trained bounded residual', 'blue', 26);
b += box(951, 710, 164, 100, 'Sum + limit', 'Combine outputs', 'orange', 22);
b += box(1182, 710, 230, 100, 'Motors + robot', 'Physical response', 'blue', 24);
b += `<rect x="64" y="977" width="1392" height="58" rx="13" fill="#F3EDFC" stroke="#DFD1F0"/>`;
b += text(760, 1014, 'LLM output: reward-design suggestions   |   Deployed controller: fixed PID + trained neural policy', 23, 600);

const artifacts = [
  ['route_a_simple', svg(880, 'Route A | LLM-Assisted Auto-Tuning', 'Harness: automated parameter proposals, validation and updates.', a, 'A real-time STM32 controller receives validated parameter updates from an automatic PC loop of telemetry, LLM proposals and validation.')],
  ['route_b_simple', svg(1064, 'Route B | LLM-Assisted Reward Design', 'Non-harness: the team reviews LLM suggestions and starts PPO training.', b, 'The team uses LLM feedback to design rewards, reviews changes, and starts PPO training. A validated neural policy is deployed alongside a fixed PID controller. Their outputs are summed and limited to control the motors. Simulation and robot logs feed the design process.')],
];
(async () => {
  for (const [name, content] of artifacts) {
    fs.writeFileSync(path.join(__dirname, `${name}.svg`), content, 'utf8');
    await sharp(Buffer.from(content), {density: 144}).png().toFile(path.join(__dirname, `${name}.png`));
    console.log(`${name}.svg + ${name}.png`);
  }
})();
