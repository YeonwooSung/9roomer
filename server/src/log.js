const express = require('express');
const router = express.Router();

router.get('/', (req, res) => {
    //TODO store log data
    res.send('hi');
});

module.exports = router;