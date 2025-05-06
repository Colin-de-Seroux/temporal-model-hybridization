from influxdb_client import InfluxDBClient

url = "http://localhost:8086"
token = "admin_token"
org = "org"
bucket = "bucket"

client = InfluxDBClient(url=url, token=token, org=org)

query_api = client.query_api()

query = f'from(bucket:"{bucket}") |> range(start: -1h)'

result = query_api.query(query)

for table in result:
    for record in table.records:
        print(f"Time: {record['_time']}, Value: {record['_value']}")
