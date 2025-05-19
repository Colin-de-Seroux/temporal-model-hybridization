# Temporal-model-hybridization

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
