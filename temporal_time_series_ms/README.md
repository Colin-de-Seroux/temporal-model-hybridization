# Temporal time series Microservice
```sh
mvn install:install-file -Dfile='src/main/libs/logger-0.0.1-SNAPSHOT.jar' -DgroupId='fr.phenix333' -DartifactId=logger -Dversion='0.0.1-SNAPSHOT' -Dpackaging=jar
```

Launch API and Postgres database with [docker_compose file (images : postgres & temporal_time_series_ms)](../docker-compose.yml).

Once the containers are running, the Swagger UI for the microservice is available at :
http://localhost:3001/swagger.html