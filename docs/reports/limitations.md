# Project limitations

The project has a modular architecture and AI integration, but it has limitation

- Prediction from AI has to be improved (add features, more different simulations, try with a bigger and different datasets ...)
- Several steps (simulation, Logs and JSON imports and CSV export in the microservice) needs to be done manually.
- Runtime in ROS 2 can be influenced by other factors like RAM, network latency, OS scheduling... that can be hard to controll (not considered yet). Simulations could be done in more different cases.
- DSL : predictions injected only for triggers and actions.
