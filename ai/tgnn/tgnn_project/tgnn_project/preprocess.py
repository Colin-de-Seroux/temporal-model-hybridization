
import pandas as pd


def encode_action_type(action_type):
    return {
        'timer': [1, 0, 0],
        'pub': [0, 1, 0],
        'sub': [0, 0, 1]
    }.get(action_type, [0, 0, 0])

def preprocess_ast_data(ast_data):
    df = pd.DataFrame(ast_data)

    df = df.rename(columns={
        'modelName': 'ModelName',
        'nodeName': 'NodeName',
        'behaviorIndex': 'BehaviorIndex',
        'actionType': 'ActionType',
        'actionOrder': 'ActionOrder',
        'topic': 'Topic',
        'value': 'Value'
    })

    print(df)
    df['Value'] = df['Value'].astype(float)
    df['ActionOrder'] = df['ActionOrder'].astype(int)
    df['BehaviorIndex'] = df['BehaviorIndex'].astype(int)


    return df