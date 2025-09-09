ssh lvuser@10.2.54.2 "ls -t /U/logs/*.wpilog | head -n 4 | tar -cf /U/logs/logdump.tar.gz -T -"
scp lvuser@10.2.54.2:/U/logs/logdump.tar.gz ~
tar -vxf ~/logdump.tar.gz
