# AI

## Random forest regressor

We cannot use a Random Forest regressor because it does not take the graph structure into account. It would be necessary to create a model derived from it to obtain coherent results.

## TGNN

[README.md](tgnn/README.md)

## Improvements

To continue the project, we'd have to create a new architecture, or at least adapt one to our requirements.

One possible solution would be to have :

- Node
  - Type < timer, pub-sub, action-service >
  - ExecutionTime
  - ConfigurationTime
- Edges between them
