grammar RosAml

entry Model:
    logger=LoggerLevel
    nodes+=Node*
    constraints+=TemporalConstraint*;


LoggerLevel:
    'logger' level=LEVEL;

Node:
    'node' name=ID 
    'expectedExecTime' expectedExecTime=DOUBLE 'ms'
    ('activationPeriod' activationPeriod=DOUBLE 'ms')?
    (callbacks+=Callback |
    publishers+=Publisher | 
    subscribers+=Subscriber | 
    services+=Service | 
    actions+=Action | 
    logs+=Logger)*;

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

Logger:
    'log' level=LEVEL msg=STRING;

Callback:
    'callback' name=ID 'on' kind=('subscriber' | 'timer' | 'service' | 'action')
    'expectedExecTime' expectedExecTime=DOUBLE 'ms';

TemporalConstraint:
    PrecedenceConstraint |
    InvariantConstraint |
    DeadlineConstraint |
    DriftDetection;

PrecedenceConstraint:
    'precedence' source=ID '->' target=ID 'within' maxDelay=DOUBLE 'ms';

InvariantConstraint:
    'invariant' invariantType=InvariantType;

InvariantType:
    MinInterEventTime | MaxInterEventTime | MaxJitter;


MinInterEventTime:
    'minInterEventTime' from=ID 'to' to=ID '>=' delay=DOUBLE 'ms';

MaxInterEventTime:
    'maxInterEventTime' from=ID 'to' to=ID '<=' delay=DOUBLE 'ms';

MaxJitter:
    'jitter' on=ID '<=' value=DOUBLE 'ms';

DeadlineConstraint:
    'deadline' on=ID '<=' deadline=DOUBLE 'ms';

DriftDetection:
    'monitorDrift'
        'on' target=ID
        'for' metric=DriftMetric
        'threshold' threshold=DOUBLE 'ms'
        ('logLevel' level=LEVEL)?;
    


terminal DriftMetric: 'execTime' | 'jitter' | 'activationPeriod' | 'interEventTime';
terminal LEVEL: "debug" | "info" | "warning" | "error";
terminal ID: /[_a-zA-Z][\w_]*/;
terminal INT returns number: /[0-9]+/;
terminal FLOAT: /[0-9]+\.[0-9]+/;
terminal DOUBLE: /([0-9]+\.[0-9]*([eE][+-]?[0-9]+)?|[0-9]+[eE][+-]?[0-9]+)/;
terminal STRING: /"(\\.|[^"\\])*"|'(\\.|[^'\\])*'/;
terminal BOOL: 'true' | 'false';

hidden terminal WS: /\s+/;
hidden terminal ML_COMMENT: /\/\*[\s\S]*?\*\//;
hidden terminal SL_COMMENT: /\/\/[^\n\r]*/;
