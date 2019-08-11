import os
import sys
out_str=""
for filename in os.listdir(sys.argv[1]):
    id=filename[0:-3]
    out_str=out_str+id+","
out_str=out_str[0:-1]
print(out_str)
