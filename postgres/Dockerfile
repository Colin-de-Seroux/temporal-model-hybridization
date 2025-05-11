FROM postgres:latest

COPY init.sh /docker-entrypoint-initdb.d/init.sh

RUN chmod +x /docker-entrypoint-initdb.d/init.sh

EXPOSE 5432
