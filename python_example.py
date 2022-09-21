
from gz.math7 import Vector3d

import sdformat as sdf

import sys

def main():
    input_file = './sphere.sdf'
    root = sdf.Root()
    try:
        root.load(input_file)
    except sdf.SDFErrorsException as e:
        print(e, file=sys.stderr)

    for world_index in range(root.world_count()):
        world = root.world_by_index(world_index)
        print(world.name())

        for model_index in range(world.model_count()):
            model = world.model_by_index(model_index)
            print("\tModel: ", model.name())
            for link_index in range(model.link_count()):
                link = model.link_by_index(link_index)
                print("\t\tLink: ", link.name())
                for collision_index in range(link.collision_count()):
                    collision = link.collision_by_index(collision_index)
                    print("\t\t\tCollision: ", collision.name())
                    for visual_index in range(link.collision_count()):
                        visual = link.visual_by_index(visual_index)
                        print("\t\t\tVisual: ", visual.name())

    # Create a new element
    world = root.world_by_index(0)
    world.set_name('shapes')

    model = sdf.Model()
    model.set_name('box')

    link = sdf.Link()
    link.set_name('box_link')

    geometry = sdf.Geometry()
    box = sdf.Box()

    box.set_size(Vector3d(1, 1, 1))
    geometry.set_box_shape(box)
    geometry.set_type(sdf.GeometryType.BOX)

    collision = sdf.Collision()
    collision.set_name('collision_box')
    collision.set_geometry(geometry)

    visual = sdf.Visual()
    visual.set_name('visual_box')
    visual.set_geometry(geometry)

    link.add_visual(visual)
    link.add_collision(collision)
    model.add_link(link)
    world.add_model(model)

    with open('new_sdf.sdf', "w") as f:
        f.write(root.to_string())


if __name__ == "__main__":
    main()
