grammar RosAml

entry Model:
    nodes+=Node*;

Node:
    'node' name=ID 
    'expectedExecTime' expectedExecTime=DOUBLE 'ms'
    (publishers+=Publisher |subscribers+=Subscriber | services+=Service | actions+=Action)*
    ;

Publisher:
    'publish' topicName=ID ('type' msgType=STRING)? 'message' msg=STRING;

Subscriber:
    'subscribe' topicName=ID ('type' msgType=STRING)? msg=STRING;

Service:
    'service' serviceName=ID ('type' srvType=STRING)?;

Action:
    'action' actionName=ID ('type' actionType=STRING)? 
    'goal' goal=STRING
    'result' result=STRING
    'feedback' feedback=STRING;


terminal ID: /[_a-zA-Z][\w_]*/;
terminal QualifiedName: ID ('.' ID)*;
terminal INT returns number: /[0-9]+/;
terminal FLOAT: /[0-9]+\.[0-9]+/;
terminal DOUBLE: /([0-9]+\.[0-9]*([eE][+-]?[0-9]+)?|[0-9]+[eE][+-]?[0-9]+)/;
terminal STRING: /"(\\.|[^"\\])*"|'(\\.|[^'\\])*'/;
terminal BOOL: 'true' | 'false';

hidden terminal WS: /\s+/;
hidden terminal ML_COMMENT: /\/\*[\s\S]*?\*\//;
hidden terminal SL_COMMENT: /\/\/[^\n\r]*/;
