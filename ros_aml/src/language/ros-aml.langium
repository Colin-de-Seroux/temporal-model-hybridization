grammar RosAml

entry Model:
    systemName=ID?
    logger=LoggerLevel?
    loopNbTime=LoopNbTime?
    nodes+=Node*
    constraints+=TemporalConstraint*
    predictedResults +=PredictedResult*;

PredictedResult:
    'predicted' predictedTime=DOUBLE;

LoggerLevel:
    'logger' level=LEVEL training="training"?;

LoopNbTime:
    'loopNbTime' value=DOUBLE;

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
    'messageReceived' topic=ID ('predicted' predictedTime=DOUBLE 'ms')?;

TimerElapsed:
    'timerElapsed' timer=ID? ('predicted' predictedTime=DOUBLE 'ms')?;

ServiceRequest:
    'serviceRequest' service=ID ('predicted' predictedTime=DOUBLE 'ms')?;

ActionGoalReceived:
    'actionGoalReceived' action=ID ('predicted' predictedTime=DOUBLE 'ms')?;

ParamChanged:
    'paramChanged' param=ID ('predicted' predictedTime=DOUBLE 'ms')?;

StateChanged:
    'stateChanged' state=ID ('predicted' predictedTime=DOUBLE 'ms')?;
Action:
    SendMessage | 
    LogMessage | 
    CallService | 
    SetParam | 
    GetParam | 
    UpdateState | 
    SendActionGoal ;

SendMessage:
    'sendMessage' topic=ID message=STRING ('qos' qos=QoS)? ('predicted' predictedTime=DOUBLE 'ms')?;

LogMessage:
    'log' level=LEVEL message=STRING ('predicted' predictedTime=DOUBLE 'ms')?;

CallService:
    'callService' service=ID (request=STRING)? expectedTime=DOUBLE 'ms' ('predicted' predictedTime=DOUBLE 'ms')?;

SetParam:
    'setParam' param=ID value=Value ('predicted' predictedTime=DOUBLE 'ms')?;

GetParam:
    'getParam' param=ID ('predicted' predictedTime=DOUBLE 'ms')?;

UpdateState:
    'updateState' state=ID value=Value ('predicted' predictedTime=DOUBLE 'ms')?;

SendActionGoal:
    'sendActionGoal' action=ID (goal=STRING)? expectedTime=DOUBLE 'ms' ('predicted' predictedTime=DOUBLE 'ms')?;

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
