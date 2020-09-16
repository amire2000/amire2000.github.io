---
layout: post
title: javascript protobuf
categories: web
tags: [javascript, protobuf]
description: Using protobuf in browser using protobuf.js library
image: 
public: true
---

# demo for protobuf.js site
- proto file
```
package awesomepackage;
syntax = "proto3";

message AwesomeMessage {
    string awesome_field = 1; // becomes awesomeField
}
```

- html / javascript file

```js
<html>

<head>
    <link rel="icon" href="data:;base64,iVBORw0KGgo=">
    <meta content="text/html;charset=utf-8" http-equiv="Content-Type">
    <meta content="utf-8" http-equiv="encoding">
    <script src="_scripts/protobuf.js"></script>
    <script>
        protobuf.load("awesome.proto", function (err, root) {
            if (err)
                throw err;

            // Obtain a message type
            var AwesomeMessage = root.lookupType("awesomepackage.AwesomeMessage");

            // Exemplary payload
            var payload = { awesomeField: "AwesomeString" };

            // Verify the payload if necessary (i.e. when possibly incomplete or invalid)
            var errMsg = AwesomeMessage.verify(payload);
            if (errMsg)
                throw Error(errMsg);

            // Create a new message
            var message = AwesomeMessage.create(payload); // or use .fromObject if conversion is necessary
            // Encode a message to an Uint8Array (browser) or Buffer (node)
            var buffer = AwesomeMessage.encode(message).finish();
            // ... do something with buffer

            // Decode an Uint8Array (browser) or Buffer (node) to a message
            var message = AwesomeMessage.decode(buffer);
            // ... do something with message

            // If the application uses length-delimited buffers, there is also encodeDelimited and decodeDelimited.

            // Maybe convert the message back to a plain object
            var object = AwesomeMessage.toObject(message, {
                longs: String,
                enums: String,
                bytes: String,
                // see ConversionOptions
            });
            console.log(object)
        });
    </script>
</head>

</html>
```

&nbsp;  
&nbsp;  
&nbsp;  
# Test


Run python html server
```bash
# python3
python -m http.server 8000
```

&nbsp;  
&nbsp;  
&nbsp;  
# Reference 
- [protobuf.js](https://protobufjs.github.io/protobuf.js/#installation)
- [Download js lib](https://github.com/protobufjs/protobuf.js/tree/master/dist)