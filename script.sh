#!/bin/bash

# Run `docker compose up` 50 times in a row
for i in {1..50}
do
    echo "Iteration $i: Starting docker compose up..."
    docker compose up
    echo "Iteration $i: Completed."
done