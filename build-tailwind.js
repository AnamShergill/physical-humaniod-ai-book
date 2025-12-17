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

// For Windows compatibility, we'll write a minimal Tailwind CSS output
// since the CLI is having issues on this system
const outputCSS = `
/* Minimal Tailwind CSS for the Physical AI textbook */
/* This includes essential utility classes */

/* Layout utilities */
.container { width: 100%; margin-right: auto; margin-left: auto; padding-right: 1rem; padding-left: 1rem; }
@media (min-width: 640px) { .container { max-width: 640px; } }
@media (min-width: 768px) { .container { max-width: 768px; } }
@media (min-width: 1024px) { .container { max-width: 1024px; } }
@media (min-width: 1280px) { .container { max-width: 1280px; } }

.flex { display: flex; }
.grid { display: grid; }
.hidden { display: none; }
.items-center { align-items: center; }
.justify-center { justify-content: center; }
.justify-between { justify-content: space-between; }
.flex-col { flex-direction: column; }
.gap-2 { gap: 0.5rem; }
.gap-3 { gap: 0.75rem; }
.gap-4 { gap: 1rem; }

/* Sizing */
.w-full { width: 100%; }
.h-full { height: 100%; }
.max-w-xs { max-width: 20rem; }
.max-w-md { max-width: 28rem; }
.max-w-3xl { max-width: 48rem; }
.max-w-7xl { max-width: 80rem; }

/* Spacing */
.p-4 { padding: 1rem; }
.p-6 { padding: 1.5rem; }
.py-2 { padding-top: 0.5rem; padding-bottom: 0.5rem; }
.py-3 { padding-top: 0.75rem; padding-bottom: 0.75rem; }
.px-3 { padding-left: 0.75rem; padding-right: 0.75rem; }
.px-4 { padding-left: 1rem; padding-right: 1rem; }
.px-6 { padding-left: 1.5rem; padding-right: 1.5rem; }
.mt-4 { margin-top: 1rem; }
.mb-2 { margin-bottom: 0.5rem; }
.mb-4 { margin-bottom: 1rem; }
.mb-6 { margin-bottom: 1.5rem; }
.mr-2 { margin-right: 0.5rem; }

/* Typography */
.text-sm { font-size: 0.875rem; line-height: 1.25rem; }
.text-base { font-size: 1rem; line-height: 1.5rem; }
.text-lg { font-size: 1.125rem; line-height: 1.75rem; }
.text-xl { font-size: 1.25rem; line-height: 1.75rem; }
.text-2xl { font-size: 1.5rem; line-height: 2rem; }
.text-3xl { font-size: 1.875rem; line-height: 2.25rem; }
.text-4xl { font-size: 2.25rem; line-height: 2.5rem; }
.font-medium { font-weight: 500; }
.font-semibold { font-weight: 600; }
.font-bold { font-weight: 700; }

/* Colors */
.text-gray-900 { color: rgb(17 24 39); }
.text-gray-800 { color: rgb(31 41 55); }
.text-gray-700 { color: rgb(55 65 81); }
.text-gray-600 { color: rgb(75 85 99); }
.text-gray-500 { color: rgb(107 114 128); }
.text-blue-600 { color: rgb(37 99 235); }
.text-green-600 { color: rgb(22 163 74); }
.text-amber-600 { color: rgb(202 138 4); }
.text-purple-800 { color: rgb(107 33 168); }

.bg-white { background-color: rgb(255 255 255); }
.bg-blue-50 { background-color: rgb(239 246 255); }
.bg-blue-100 { background-color: rgb(219 234 254); }
.bg-blue-600 { background-color: rgb(37 99 235); }
.bg-teal-50 { background-color: rgb(240 253 250); }
.bg-gray-50 { background-color: rgb(249 250 251); }
.bg-gray-100 { background-color: rgb(243 244 246); }
.bg-green-50 { background-color: rgb(240 253 244); }
.bg-amber-50 { background-color: rgb(255 251 235); }
.bg-purple-50 { background-color: rgb(250 245 255); }

/* Borders */
.border { border-width: 1px; }
.border-0 { border-width: 0px; }
.border-l-4 { border-left-width: 4px; }
.border-gray-200 { border-color: rgb(229 231 235); }
.border-blue-200 { border-color: rgb(191 219 254); }
.border-blue-500 { border-color: rgb(59 130 246); }
.rounded-lg { border-radius: 0.5rem; }
.rounded-xl { border-radius: 0.75rem; }
.rounded-full { border-radius: 9999px; }

/* Effects */
.shadow-sm { box-shadow: 0 1px 2px 0 rgb(0 0 0 / 0.05); }
.shadow-md { box-shadow: 0 4px 6px -1px rgb(0 0 0 / 0.1), 0 2px 4px -2px rgb(0 0 0 / 0.1); }
.shadow-lg { box-shadow: 0 10px 15px -3px rgb(0 0 0 / 0.1), 0 4px 6px -4px rgb(0 0 0 / 0.1); }

/* Interactive */
.cursor-pointer { cursor: pointer; }
.cursor-default { cursor: default; }

/* Hover states */
.hover\\:bg-blue-50:hover { background-color: rgb(239 246 255); }
.hover\\:bg-blue-700:hover { background-color: rgb(29 78 216); }
.hover\\:shadow-md:hover { box-shadow: 0 4px 6px -1px rgb(0 0 0 / 0.1), 0 2px 4px -2px rgb(0 0 0 / 0.1); }

/* Responsive */
@media (min-width: 768px) {
  .md\\:flex { display: flex; }
  .md\\:flex-row { flex-direction: row; }
  .md\\:items-center { align-items: center; }
  .md\\:justify-between { justify-content: space-between; }
  .md\\:max-w-md { max-width: 28rem; }
}

/* Additional custom styles for our components */
.card { background-color: rgb(255 255 255); border-radius: 0.75rem; box-shadow: 0 1px 2px 0 rgb(0 0 0 / 0.05); border: 1px solid rgb(229 231 235); transition: all 150ms cubic-bezier(0.4, 0, 0.2, 1); }
.btn-primary { background-color: rgb(37 99 235); color: rgb(255 255 255); padding: 0.75rem 1.5rem; border-radius: 0.5rem; font-weight: 500; transition: all 150ms cubic-bezier(0.4, 0, 0.2, 1); }
`;

const outputPath = path.join(__dirname, 'src', 'css', 'tailwind.css');
fs.writeFileSync(outputPath, outputCSS);

console.log('Tailwind CSS file created successfully!');

// Clean up temporary file
fs.unlinkSync(tempPath);

