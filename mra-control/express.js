const express = require('express');
const cors = require('cors');

const app = express();

app.use(cors());  // Enable CORS for all routes

// Your routes and application logic here

app.listen(3000, () => {
  console.log('Server started on port 3000');
});
