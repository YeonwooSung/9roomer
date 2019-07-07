const express = require('express');
const router = express.Router();

router.get('/', (req, res) => {
    //get current time
    res.send('hi');
});

module.exports = router;