grammar RosAml

entry Model:
    nodes+=Node*;

Node:
    'node' name=ID 
    'expectedExecTime' expectedExecTime=DOUBLE 'ms'
    "["
        (publishers+=Publisher |subscribers+=Subscriber | services+=Service)*
    "]";

Publisher:
    'publish' topicName=ID 'type' msgType=ID;

Subscriber:
    'subscribe' topicName=ID 'type' msgType=ID;

Service:
    'service' serviceName=ID 'ofType' srvType=ID;


terminal ID: /[_a-zA-Z][\w_]*/;
terminal INT returns number: /[0-9]+/;
terminal FLOAT: /[0-9]+\.[0-9]+/;
terminal DOUBLE: /([0-9]+\.[0-9]*([eE][+-]?[0-9]+)?|[0-9]+[eE][+-]?[0-9]+)/;
terminal STRING: /"(\\.|[^"\\])*"|'(\\.|[^'\\])*'/;
terminal BOOL: 'true' | 'false';

hidden terminal WS: /\s+/;
hidden terminal ML_COMMENT: /\/\*[\s\S]*?\*\//;
hidden terminal SL_COMMENT: /\/\/[^\n\r]*/;
