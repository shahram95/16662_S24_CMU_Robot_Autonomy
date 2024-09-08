import mujoco

for attribute_name in dir(mujoco):
    attribute = getattr(mujoco, attribute_name)
    print(f"Name: {attribute_name}")
    print(f"Docstring: {attribute.__doc__}")
    print("\n")
