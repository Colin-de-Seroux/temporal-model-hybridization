grammar RosAml

entry Model:
    systemName=ID?
    logger=LoggerLevel?
    nodes+=Node*
    constraints+=TemporalConstraint*;

LoggerLevel:
    'logger' level=LEVEL;

Node:
    'node' name=ID
    'expectedExecTime' expectedExecTime=DOUBLE 'ms'
    ('activation' activation=ActivationPattern)?
    params+=Parameter*
    states+=State*
    timers+=Timer*
    behaviors+=Behavior*;

Parameter:
    'param' name=ID ('default' defaultValue=Value)?;

State:
    'state' name=ID ('initial' initialValue=Value)?;

Timer:
    'timer' name=ID 'period' period=DOUBLE 'ms';

ActivationPattern:
    'periodic' timer=ID |
    'event-driven' source=ID;

Behavior:
    'when' trigger=Trigger 'do' action+=Action+;

Trigger:
    MessageReceived |
    TimerElapsed |
    ServiceRequest |
    ActionGoalReceived |
    ParamChanged |
    StateChanged;

MessageReceived:
    'messageReceived' topic=ID;

TimerElapsed:
    'timerElapsed' timer=ID?;

ServiceRequest:
    'serviceRequest' service=ID;

ActionGoalReceived:
    'actionGoalReceived' action=ID;

ParamChanged:
    'paramChanged' param=ID;

StateChanged:
    'stateChanged' state=ID;
Action:
    SendMessage | 
    LogMessage | 
    CallService | 
    SetParam | 
    GetParam | 
    UpdateState | 
    SendActionGoal ;

SendMessage:
    'sendMessage' topic=ID message=STRING ('qos' qos=QoS)?;

LogMessage:
    'log' level=LEVEL message=STRING;

CallService:
    'callService' service=ID (request=STRING)? expectedTime=DOUBLE 'ms';

SetParam:
    'setParam' param=ID value=Value;

GetParam:
    'getParam' param=ID;

UpdateState:
    'updateState' state=ID value=Value;

SendActionGoal:
    'sendActionGoal' action=ID (goal=STRING)? expectedTime=DOUBLE 'ms';

Value:
    intValue=INT | doubleValue=DOUBLE| boolValue=BOOL | stringValue=STRING ;

QoS:
    reliable='reliable' | 
    bestEffort='bestEffort';

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
terminal DOUBLE : /[0-9]+(\.[0-9]+)?([eE][+-]?[0-9]+)?/;
terminal INT returns number: /[0-9]+/;
terminal BOOL : 'true' | 'false';
terminal STRING returns string: /"(\\.|[^"\\])*"|'(\\.|[^'\\])*'/;


hidden terminal WS: /\s+/;
hidden terminal ML_COMMENT: /\/\*[\s\S]*?\*\//;
hidden terminal SL_COMMENT: /\/\/[^\n\r]*/;
