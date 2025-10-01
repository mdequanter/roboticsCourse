# Python code om config te lezen
with open("python3/config.txt") as f:
    for line in f:
        if line.startswith("#"):
            continue
        key, value = line.strip().split("=")
        print(key, "=", value)
