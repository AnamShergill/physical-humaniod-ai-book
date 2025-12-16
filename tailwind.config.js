/** @type {import('tailwindcss').Config} */
module.exports = {
  content: [
    "./src/**/*.{js,jsx,ts,tsx,md,mdx}",
    "./docs/**/*.{md,mdx}",
    "./blog/**/*.{md,mdx}",
    "./pages/**/*.{js,jsx,ts,tsx,md,mdx}",
    "./theme/**/*.{js,jsx,ts,tsx}",
    "./node_modules/docusaurus-theme-search/node_modules/@docsearch/react/dist/esm/**/*.js",
  ],
  theme: {
    extend: {
      colors: {
        primary: {
          50: '#eff6ff',
          500: '#3b82f6',
          600: '#1e3a8a',
          700: '#1d4ed8',
        },
        secondary: {
          500: '#0f766e',
          600: '#047857',
        },
      },
    },
  },
  plugins: [],
}