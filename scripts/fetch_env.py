import os

# import pio environment
Import("env")

try:
    from dotenv import dotenv_values
except ImportError:
    env.Execute("$PYTHONEXE -m pip install python-dotenv")
    # import again
    from dotenv import dotenv_values

config = dotenv_values(".env")

flags = []

for k, v in config.items():
    flags.append("-D " + k + "='\"" + v + "\"'")

print(flags)

env.Append(BUILD_FLAGS=flags)
