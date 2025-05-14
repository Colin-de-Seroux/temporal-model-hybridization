grammar RosAml

entry Model:
    logger=LoggerLevel

    nodes+=Node*;


LoggerLevel:
    'logger' level=LEVEL;

Node:
    'node' name=STRING expectedExecTime=DOUBLE
    init=Steps?
    (callbackFunctions+=CallbackFunction | services+=Service | actions+=Action)*;

Steps:
    'steps' '{' steps+=Step* '}';

Step:
    Logger | Publisher | Subscriber | Timer | Service | Action | Publish;

Logger:
    'log' level=LEVEL msg=STRING;

Publisher:
    'publisher' name=STRING msgType=MSGTYPE topic=STRING qos=INT;

Subscriber:
    'subscriber' name=STRING msgType=MSGTYPE topic=STRING callbackFunction=STRING qos=INT;

Timer:
    'timer' name=STRING period=DOUBLE callbackFunction=STRING;

CallbackFunction:
    'callbackFunction' name=STRING type=CFTYPE
    steps=Steps?;

Service:
    'service' name=STRING estimatedExecutionTime=DOUBLE
    (logs+=Logger)*;

Action:
    'action' name=STRING estimatedExecutionTime=DOUBLE
    (logs+=Logger)*;

Publish:
    'publish' publisherName=STRING msg=STRING;


terminal LEVEL: 'debug' | 'info' | 'warning' | 'error';
terminal MSGTYPE: 'String' | 'Int32' | 'Float32' | 'Bool';
terminal CFTYPE: 'Publisher' | 'Subscriber';
terminal DOUBLE: /([0-9]+\.[0-9]*([eE][+-]?[0-9]+)?|[0-9]+[eE][+-]?[0-9]+)/;
terminal INT returns number: /[0-9]+/;
terminal STRING: /"(\\.|[^"\\])*"|'(\\.|[^'\\])*'/;

hidden terminal WS: /\s+/;
hidden terminal ML_COMMENT: /\/\*[\s\S]*?\*\//;
hidden terminal SL_COMMENT: /\/\/[^\n\r]*/;
