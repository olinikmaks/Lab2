import NemAll_Python_Geometry as Geo
import NemAll_Python_BaseElements as base
import NemAll_Python_BasisElements as basis
import NemAll_Python_Utility as Util
import GeometryValidate as Validator
from HandleDirection import HandleDirection
from HandleProperties import HandleProperties


# Define version of allplan
def check_allplan_version(build_ele, version):
    del build_ele
    del version
    return True


# Create object of beam or another figure
def create_element(build_ele, doc):
    element = SimpleBeam(doc)
    return element.create(build_ele)


# Handles
def move_handle(build_ele, handle_prop, input_pnt, doc):
    build_ele.change_property(handle_prop, input_pnt)
    return create_element(build_ele, doc)


# Our Beam
class SimpleBeam:

    def __init__(self, doc):
        self.model_ele_list = []
        self.handle_list = []
        self.document = doc

    def create(self, build_ele):
        self.beam_upper_part(build_ele)
        self.handles(build_ele)
        return (self.model_ele_list, self.handle_list)

    def checker_bottom(shape, shape2, widc, botc):
        if widc > 0:
            shape_e = Util.VecSizeTList()
            shape_e.append(1)
            shape_e.append(3)
            _, shape = Geo.ChamferCalculus.Calculate(
                shape, shape_e, widc, False)

            if not Validator.polyhedron(_):
                return [],[]

        if botc > 0:
            shape_e2 = Util.VecSizeTList()
            shape_e2.append(8)
            shape_e2.append(10)
            _, shape2 = Geo.ChamferCalculus.Calculate(
                shape2, shape_e2, botc, False)

            if not Validator.polyhedron(_):
                return [],[]
        return shape, shape2

    def checker_upper(shape, top):
        if top > 0:
            shape_e2 = Util.VecSizeTList()
            shape_e2.append(8)
            shape_e2.append(10)
            _, shape = Geo.ChamferCalculus.Calculate(
                shape, shape_e2, top, False)

            if not Validator.polyhedron(_):
                return

    def beam_lower_part(self, build_ele):
        data = self.get_interface_data(build_ele)
        shape = Geo.BRep3D.CreateCuboid(Geo.AxisPlacement3D(Geo.Point3D(0, 0, 0), Geo.Vector3D(
            1, 0, 0), Geo.Vector3D(0, 0, 1)), data[0], data[1], data[2])
        shape2 = Geo.BRep3D.CreateCuboid(Geo.AxisPlacement3D(Geo.Point3D(0, 0, 0), Geo.Vector3D(
            1, 0, 0), Geo.Vector3D(0, 0, 1)), data[0], data[1], data[2])
        widc = build_ele.cut_bottom_top.value
        botc = build_ele.sectionb.value
        shape, shape2 = self.checker_bottom(shape, shape2, widc, botc)
        _, eshape = Geo.MakeIntersection(shape, shape2)
        return eshape

    def beam_middle_part(self, build_ele):
        data = self.get_interface_data(build_ele)
        shape = Geo.BRep3D.CreateCuboid(Geo.AxisPlacement3D(Geo.Point3D(
            data[0] / 2 - data[3] / 2, 0, data[2]), Geo.Vector3D(1, 0, 0), Geo.Vector3D(0, 0, 1)), data[3], data[1], data[4])
        shape1 = Geo.BRep3D.CreateCylinder(Geo.AxisPlacement3D(Geo.Point3D(
            data[5], data[1] / 8, data[2] + data[4] / 2), Geo.Vector3D(0, 0, 1), Geo.Vector3D(1, 0, 0)), data[6], data[3])
        shape2 = Geo.BRep3D.CreateCylinder(Geo.AxisPlacement3D(Geo.Point3D(
            data[5], data[1] - data[1] / 8, data[2] + data[4] / 2), Geo.Vector3D(0, 0, 1), Geo.Vector3D(1, 0, 0)), data[6], data[3])
        _, shape = Geo.MakeSubtraction(shape, shape1)
        _, shape = Geo.MakeSubtraction(shape, shape2)
        _, eshape = Geo.MakeUnion(
            shape, self.beam_lower_part(build_ele))
        return eshape

    def beam_upper_part(self, build_ele):
        data = self.get_interface_data(build_ele)
        shape = Geo.BRep3D.CreateCuboid(Geo.AxisPlacement3D(Geo.Point3D(
            0 - (data[7] - data[0]) / 2, 0, data[2] + data[4]), Geo.Vector3D(1, 0, 0), Geo.Vector3D(0, 0, 1)), data[7], data[1], data[8])
        pl = Geo.BRep3D.CreateCuboid(Geo.AxisPlacement3D(Geo.Point3D(data[9] - (data[7] - data[0]) / 2, 0, data[2] + data[4] + data[8]), Geo.Vector3D(
            1, 0, 0), Geo.Vector3D(0, 0, 1)), data[7] - data[9]*2, data[1], data[10])
        com_prop = base.CommonProperties()
        com_prop.GetGlobalProperties()
        com_prop.Pen = 1
        com_prop.Color = data[11]
        top = data[12]
        shape = self.checker_upper(shape, top)
        _, eshape = Geo.MakeUnion(shape, self.beam_middle_part(build_ele))
        _, eshape = Geo.MakeUnion(eshape, pl)
        self.model_ele_list.append(
            basis.ModelElement3D(com_prop, eshape))

    def handles(self, build_ele):
        data = self.get_interface_data(build_ele)
        handle_1 = Geo.Point3D(
            data[0] / 2, data[1], data[4] + data[2])
        self.handle_list.append(HandleProperties("center_height", Geo.Point3D(handle_1.X, handle_1.Y, handle_1.Z), Geo.Point3D(
            handle_1.X, handle_1.Y, handle_1.Z - data[4]), [("center_height", HandleDirection.z_dir)], HandleDirection.z_dir, False))
        handle_2 = Geo.Point3D(data[0] / 2, 0, data[2] / 2)
        self.handle_list.append(HandleProperties("length", Geo.Point3D(handle_2.X, handle_2.Y + data[1], handle_2.Z), Geo.Point3D(
            handle_2.X, handle_2.Y, handle_2.Z), [("length", HandleDirection.y_dir)], HandleDirection.y_dir, False))
        handle_3 = Geo.Point3D(
            0, data[1], (data[2] - data[5]) / 2)
        self.handle_list.append(HandleProperties("width_bottom", Geo.Point3D(handle_3.X + data[0], handle_3.Y, handle_3.Z), Geo.Point3D(
            handle_3.X, handle_3.Y, handle_3.Z), [("width_bottom", HandleDirection.x_dir)], HandleDirection.x_dir, False))
        handle_4 = Geo.Point3D(0 - (data[7] - data[0]) / 2,
                                    data[1], data[4] + data[2] + data[12])
        self.handle_list.append(HandleProperties("upper_width", Geo.Point3D(handle_4.X + data[7], handle_4.Y, handle_4.Z), Geo.Point3D(
            handle_4.X, handle_4.Y, handle_4.Z), [("upper_width", HandleDirection.x_dir)], HandleDirection.x_dir, False))
        handle_5 = Geo.Point3D(data[0] / 2, data[1],
                                    data[4] + data[2] - data[2] / 4)
        self.handle_list.append(HandleProperties("upper_height", Geo.Point3D(handle_5.X, handle_5.Y, handle_5.Z + data[8]), Geo.Point3D(
            handle_5.X, handle_5.Y, handle_5.Z), [("upper_height", HandleDirection.z_dir)], HandleDirection.z_dir, False))
        handle_6 = Geo.Point3D(data[0] / 2, data[1],
                                    data[4] + data[2] + data[8])
        self.handle_list.append(HandleProperties("height_plate", Geo.Point3D(handle_6.X, handle_6.Y, handle_6.Z + data[10]), Geo.Point3D(
            handle_6.X, handle_6.Y, handle_6.Z), [("height_plate", HandleDirection.z_dir)], HandleDirection.z_dir, False))
        handle_7 = Geo.Point3D(data[0] / 2, data[1], 0)
        self.handle_list.append(HandleProperties("height_bottom", Geo.Point3D(handle_7.X, handle_7.Y, handle_7.Z + data[2]), Geo.Point3D(
            handle_7.X, handle_7.Y, handle_7.Z), [("height_bottom", HandleDirection.z_dir)], HandleDirection.z_dir, False))
        handle_8 = Geo.Point3D(data[0] / 2 - data[3] / 2,
                                    data[1], data[4] / 2 + data[2])
        self.handle_list.append(HandleProperties("width_center", Geo.Point3D(handle_8.X + data[3], handle_8.Y, handle_8.Z), Geo.Point3D(
            handle_8.X, handle_8.Y, handle_8.Z), [("width_center", HandleDirection.x_dir)], HandleDirection.x_dir, False))

    def get_interface_data(self, build_ele):
        width_bottom = build_ele.width_bottom.value
        length = build_ele.length.value
        height_bottom = build_ele.height_bottom.value
        width_center = build_ele.width_center.value
        center_height = build_ele.center_height.value
        cut_bottom_top = build_ele.cut_bottom_top.value
        radius = build_ele.radius.value
        upper_width = build_ele.upper_width.value
        upper_height = build_ele.upper_height.value
        plate_cut = build_ele.plate_cut.value
        height_plate = build_ele.height_plate.value
        color = build_ele.color.value
        upper_cut_top = build_ele.upper_cut_top.value
        return width_bottom, length, height_bottom, width_center, center_height, cut_bottom_top, radius, upper_width, upper_height, plate_cut, height_plate, color, upper_cut_top
