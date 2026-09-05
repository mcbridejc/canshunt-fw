#!bash
zencan-cli -c "nmt reset-app 10" can0
zencan-cli -c "load-config 10 configs/output_raw_tpdo.toml" can0
zencan-cli -c "nmt start 10" can0
