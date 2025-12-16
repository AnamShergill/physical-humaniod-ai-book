module.exports = {
  root: true,
  extends: [
    '@docusaurus',
    'prettier',
    'prettier/react',
  ],
  plugins: [
    'prettier',
  ],
  rules: {
    'prettier/prettier': 'error',
  },
};