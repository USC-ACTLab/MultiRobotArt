/** @type {import('tailwindcss').Config} */
module.exports = {
  content: ['./index.html', './src/**/*.{js,ts,jsx,tsx}', 'node_modules/flowbite-react/**/*.{js,jsx,ts,tsx}'],
  theme: {
    colors: {
      bl: '#5fc1b6',
      gr: '#ebebeb',
      white: '#ffffff',
      re: '#d5373e',
      ye: '#f2a140',
    },
  },
  plugins: [require('flowbite/plugin')],
};
