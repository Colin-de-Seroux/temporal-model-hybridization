#!/bin/bash
set -e

if [ -z "$(ls -A "$PGDATA")" ]; then
    echo "Database directory is empty. Creating default database..."

    psql -v ON_ERROR_STOP=1 --username "$POSTGRES_USER" <<-EOSQL
    CREATE DATABASE "$POSTGRES_DB";
EOSQL
else
    echo "Database already initialized. Skipping init script."
fi
