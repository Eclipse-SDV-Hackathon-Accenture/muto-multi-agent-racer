# Building ROS Apps and Services with Eclipse Zenoh

- You can use multi-agent race as a data provider for Fleet Management System (FMS).
- Each racer sends ROS messages such as vehicle speed, steering angle and RPM as VSS signals.
    - Vehicle.Speed
    - Vehicle.Tacograph.VehicleSpeed
    - Vehicle.Chassis.SteeringWheel.Angle
    - Vehicle.Powertrain.CombustionEngine.Speed

## Quickstart

- Clone fleet-management repository inside top level folder.
```sh
git clone https://github.com/Eclipse-SDV-Hackathon-Accenture/fleet-management.git
```
- Create a new docker compose file (`fleet-management/fms-blueprint-compose-muto.yaml`) inside fleet-management as follows:

```yaml
services:
  databroker-racer:
    image: "ghcr.io/eclipse/kuksa.val/databroker:0.4.1"
    container_name: "databroker-racer"
    networks:
    - "fms-vehicle"
    # ports:
    # - "127.0.0.1:55555:55556"
    configs:
    - "vss_overlay.json"
    environment:
      KUKSA_DATA_BROKER_ADDR: "0.0.0.0"
      KUKSA_DATA_BROKER_PORT: "55556"
      KUKSA_DATA_BROKER_METADATA_FILE: "/vss_overlay.json"
      RUST_LOG: "info"
    # for the time being, we do not use TLS secured connections to Databroker
    command: "--insecure"
  fms-forwarder-racer:
    image: "ghcr.io/eclipse-sdv-blueprints/fleet-management/fms-forwarder:main"
    build: &fms-forwarder-build
      context: "./components"
      dockerfile: "Dockerfile.fms-forwarder"
    container_name: "fms-forwarder-racer"
    networks:
    - "fms-backend"
    - "fms-vehicle"
    depends_on:
      influxdb:
        condition: service_healthy
      databroker:
        condition: service_started
    command: "influx"
    env_file: "${FMS_FORWARDER_PROPERTIES_FILE:-./influxdb/fms-demo.env}"
    environment:
      INFLUXDB_TOKEN_FILE: "/etc/forwarder/fms-demo.token"
      KUKSA_DATA_BROKER_URI: "http://databroker-racer:55556"
      RUST_LOG: "${FMS_FORWARDER_LOG_CONFIG:-info,fms_forwarder=info,influx_client=info}"
      TRUST_STORE_PATH: "${FMS_FORWARDER_TRUST_STORE_PATH:-/etc/ssl/certs/ca-certificates.crt}"
    volumes:
    - type: "volume"
      source: "influxdb-auth"
      target: "/etc/forwarder"
      read_only: true
```

- Start fleet-management, using the fms docker compose file provided in this repository  (`fms-docker-compose.yaml`) as follows:

```sh
docker compose -f ./fms-docker-compose.yaml up
```

- You can change parameters of the racer data provider from `samples/fms/fms.yaml`

- You can refer to the previous steps to start the racer.

- Your data will start showing up, in the graphana dashboard w.r.t VIN provided in the config file.
