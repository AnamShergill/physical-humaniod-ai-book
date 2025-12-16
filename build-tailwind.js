const { spawn } = require('child_process');
const fs = require('fs');
const path = require('path');

// Create a temporary CSS file with Tailwind directives
const tempCSS = `
@tailwind base;
@tailwind components;
@tailwind utilities;
`;

const tempPath = path.join(__dirname, 'temp-input.css');
fs.writeFileSync(tempPath, tempCSS);

// Run Tailwind CSS CLI to build the output
const tailwindArgs = [
  '-i', tempPath,
  '-o', path.join(__dirname, 'src/css/tailwind-output.css'),
  '--minify'
];

console.log('Building Tailwind CSS...');
const buildProcess = spawn('npx', ['tailwindcss', ...tailwindArgs], {
  stdio: 'inherit',
  shell: true
});

buildProcess.on('close', (code) => {
  // Clean up temporary file
  fs.unlinkSync(tempPath);
  console.log(`Tailwind CSS build completed with code ${code}`);
});