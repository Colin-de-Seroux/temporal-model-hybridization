# Temporal-model-hybridization

##  DSL

```bash
cd ros_aml
npm install
```
Make 1. and 2. in one command
```bash
npm run all 
```
Exemple of **.rosaml** files can be found in `ros_configs`

Generated packages can be found in `ros2\src`

**OR :**

1. To generate ast & build the DSL
```bash
npm run langium:generate

npm run build
```
2. To generate all `.rosaml` config files from `ros_configs`

```bash
npm run cli:all
```

To generate one file 
```bash
npm run cli ..\ros_packages\your_file.rosaml
```


## Environment

Copy `.env.example` in `.env`.

## RUN

```bash
docker compose up -d --build
```

## Issues

> [!Warning]
> If you have an error like this:

```sh
ERROR [test internal] load build context                                                                                                                                                                                                                                                 0.0s
 => => transferring context: 11.12kB                                                                                                                                                                                                                                                         0.0s
------
[+] Running 0/1al] load build context:
 - Service test  Building                                                                                                                                                                                                                                                                    0.8s
failed to solve: archive/tar: unknown file mode ?rwxr-xr-x
```

Delete Logs.
