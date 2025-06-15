# Temporal time series micro service

## Run locally

```sh
mvn install:install-file -Dfile='src/main/libs/logger-0.0.1-SNAPSHOT.jar' -DgroupId='fr.phenix333' -DartifactId=logger -Dversion='0.0.1-SNAPSHOT' -Dpackaging=jar
```

```sh
mvn spring-boot:run
```

## Swagger

[Swagger locally](./src/main/resources/static/swagger.yml) or [Swagger in web](http://loaclhost:3001/swagger.html)
