FROM maven:latest AS build

WORKDIR /app

COPY . .

RUN mvn install:install-file -Dfile='src/main/libs/logger-0.0.1-SNAPSHOT.jar' -DgroupId='fr.phenix333' -DartifactId=logger -Dversion='0.0.1-SNAPSHOT' -Dpackaging=jar

RUN mvn clean install -DskipTests

FROM openjdk:17-jdk-slim

WORKDIR /app

RUN mkdir Logs

COPY --from=build /app/target/temporal-time-series-ms-0.0.1-SNAPSHOT.jar .

CMD ["java", "-jar", "temporal-time-series-ms-0.0.1-SNAPSHOT.jar"]
