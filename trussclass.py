###########################################
###  Definition of a class named Truss 
###  it will generate geometry and basic information of a truss.
###########################################


import math

#   o-----o [col{q}_joint0]
#  /     /|
# o-----o |
# |\   /|\o [col{q}_diaph0]
# | \ / |/|  
# o--o--o |  [cross{q}_0]
# | / \ |\o [col{q}_base]
# |/   \|/
# o     o 

class Vec:
    @staticmethod
    def add(x, y):
        return (x[0] + y[0], x[1] + y[1], x[2] + y[2])

    @staticmethod
    def minus(x, y):
        return (x[0] - y[0], x[1] - y[1], x[2] - y[2])

    @staticmethod
    def amp(x, y):
        return (x[0] * y, x[1] * y, x[2] * y)

    @staticmethod
    def len(x):
        return (x[0] ** 2 + x[1] ** 2 + x[2] ** 2) ** 0.5

    @staticmethod
    def scale_to(x, y):
        return Vec.amp(x, y / Vec.len(x))

    @staticmethod
    def mid(x, y):
        return (0.5 * (x[0] + y[0]), 0.5 * (x[1] + y[1]), 0.5 * (x[2] + y[2]))
    
    @staticmethod
    def dot(x, y):
        return x[0] * y[0] + x[1] * y[1] + x[2] * y[2]
    
    @staticmethod
    def angle(x, y):
        cs = Vec.dot(x, y) / Vec.len(x) / Vec.len(y)
        return math.acos(cs)
    
    @staticmethod
    def mul(x, y):
        return (x[1] * y[2] - x[2] * y[1], x[2] * y[0] - x[0] * y[2], x[0] * y[1] - x[1] * y[0])
    
    @staticmethod
    def solve_pt(x0, x1, y1, x2, y2):
        k = (y2 - y1) / (x2 - x1)
        y0 = k * (x0 - x1) + y1
        return y0

class Truss:
    def __init__(self, top_ele, btm_span, top_span, coltop_ele, joint_eles, col_secs, diag_secs, hor_secs, diaph_secs):
        self.top_ele = top_ele
        self.btm_span = btm_span
        self.top_span = top_span
        self.coltop_ele = coltop_ele
        self.joint_eles = joint_eles
        self.col_secs = col_secs
        self.diag_secs = diag_secs
        self.hor_secs = hor_secs
        self.diaph_secs = diaph_secs
        self.nodes = self.get_nodes()
        self.elements = self.get_elements()
        self.sections = self.get_sections()
        self.axes = self.get_axes()

    @property
    def num_segments(self):
        return len(self.joint_eles)

    @property
    def btm_circumrad(self):
        return self.btm_span / math.sqrt(2)

    @property
    def top_circumrad(self):
        return self.top_span / math.sqrt(2)

    @property
    def col_tan(self):
        return self.top_ele / (self.btm_circumrad - self.top_circumrad)

    @property
    def col_angle(self):
        return math.atan(self.col_tan)
    
    @property
    def top_diag_angle(self):
        nodes = self.nodes
        i = self.num_segments
        p0 = nodes["col1_joint%d" % (i-1)]
        p1 = nodes["col1_joint%d" % (i-2)]
        p2 = nodes["cross1_%d" % (i-1)]
        vec1 = Vec.minus(p1, p0)
        vec2 = Vec.minus(p2, p0)
        return Vec.angle(vec1, vec2)

    @property
    def plane_angle(self):
        return math.atan(self.col_tan * math.sqrt(2))

    @property
    def joint_circumrads(self):
        return [self.ele2circumrad(ele) for ele in self.joint_eles]

    @property
    def joint_inters(self):
        return [self.ele2inter(ele) for ele in self.joint_eles]

    @property
    def diaphragm_eles(self):
        res = []
        for i in range(self.num_segments):
            upper_circumrad = self.joint_circumrads[i]
            lower_circumrad = (
                self.joint_circumrads[i - 1] if i != 0 else self.btm_circumrad
            )
            height = (
                self.joint_eles[i] - self.joint_eles[i - 1]
                if i != 0
                else self.joint_eles[i]
            )
            offset_diaphragm = (
                lower_circumrad / (lower_circumrad + upper_circumrad) * height
            )
            ele_diaphragm = (
                self.joint_eles[i - 1] + offset_diaphragm
                if i != 0
                else offset_diaphragm
            )
            res.append(ele_diaphragm)
        return res

    @property
    def diaphragm_inters(self):
        return [self.ele2inter(ele) for ele in self.diaphragm_eles]

    def ele2circumrad(self, ele):
        return self.btm_circumrad - ele / self.col_tan

    def ele2inter(self, ele):
        return self.ele2circumrad(ele) * math.sqrt(2) / 2

    @staticmethod
    def rot_quad(coords, quad):
        if quad == 1:
            return (abs(coords[0]), abs(coords[1]), coords[2])
        elif quad == 2:
            return (-abs(coords[1]), abs(coords[0]), coords[2])
        elif quad == 3:
            return (-abs(coords[0]), -abs(coords[1]), coords[2])
        elif quad == 4:
            return (abs(coords[1]), -abs(coords[0]), coords[2])
        else:
            raise Exception("Wrong quad value when calling rot_quad()")

    def get_nodes(self):
        nodes = {}
        for q in range(1, 5):
            for i in range(self.num_segments):
                nodes["col%d_joint%d" % (q, i)] = Truss.rot_quad(
                    (self.joint_inters[i], self.joint_inters[i], self.joint_eles[i]), q
                )
            for i in range(self.num_segments):
                nodes["col%d_diaph%d" % (q, i)] = Truss.rot_quad(
                    (
                        self.diaphragm_inters[i],
                        self.diaphragm_inters[i],
                        self.diaphragm_eles[i],
                    ),
                    q,
                )
            intercept_colbase = self.ele2inter(0.0)
            nodes["col%d_base" % q] = Truss.rot_quad(
                (intercept_colbase, intercept_colbase, 0.0), q
            )
            intercept_coltop = self.ele2inter(self.coltop_ele)
            nodes["col%d_top" % q] = Truss.rot_quad(
                (intercept_coltop, intercept_coltop, self.coltop_ele), q
            )
        for q in range(1, 5):
            for i in range(self.num_segments):
                nodes["cross%d_%d" % (q, i)] = Truss.rot_quad(
                    (self.diaphragm_inters[i], 0.0, self.diaphragm_eles[i]), q
                )
            i = self.num_segments
            nodes["cross%d_%d" % (q, i)] = Truss.rot_quad(
                (self.joint_inters[i - 1], 0.0, self.joint_eles[i - 1]), q
            )
        return nodes

    def get_elements(self):
        eles = {}
        for q in range(1, 5):
            q_next = 1 if q == 4 else q + 1
            for i in range(self.num_segments):
                eles["col%d_seg%d" % (q, 2*i)] = (
                    "col%d_joint%d" % (q, i-1) if i != 0 else "col%d_base" % q,
                    "col%d_diaph%d" % (q, i),
                )
                eles["col%d_seg%d" % (q, 2*i+1)] = (
                    "col%d_diaph%d" % (q, i),
                    "col%d_joint%d" % (q, i),
                )
            i = self.num_segments
            eles["col%d_seg%d" % (q, 2*i)] = ("col%d_joint%d" % (q, i-1), "col%d_top" % q)
            for i in range(self.num_segments):
                eles["diag%d_lowa%d" % (q, i)] = (
                    "cross%d_%d" % (q, i),
                    "col%d_joint%d" % (q, i-1) if i != 0 else "col%d_base" % q,
                )
                eles["diag%d_lowb%d" % (q, i)] = (
                    "col%d_joint%d" % (q, i-1) if i != 0 else "col%d_base" % q,
                    "cross%d_%d" % (q_next, i),
                )
                eles["diag%d_upa%d" % (q, i)] = ("cross%d_%d" % (q, i), "col%d_joint%d" % (q, i))
                eles["diag%d_upb%d" % (q, i)] = ("col%d_joint%d" % (q, i), "cross%d_%d" % (q_next, i))
                eles["hor%d_a%d" % (q, i)] = ("cross%d_%d" % (q, i), "col%d_diaph%d" % (q, i))
                eles["hor%d_b%d" % (q, i)] = ("col%d_diaph%d" % (q, i), "cross%d_%d" % (q_next, i))
                eles["diaph%d_%d" % (q, i)] = ("cross%d_%d" % (q, i), "cross%d_%d" % (q_next, i))
            i = self.num_segments
            eles["hor%d_a%d" % (q, i)] = ("cross%d_%d" % (q, i), "col%d_joint%d" % (q, i-1))
            eles["hor%d_b%d" % (q, i)] = ("col%d_joint%d" % (q, i-1), "cross%d_%d" % (q_next, i))
        return eles

    def get_sections(self):
        secs = {}
        for q in range(1, 5):
            for i in range(self.num_segments):
                secs["col%d_seg%d" % (q, 2*i)] = self.col_secs[2 * i]
                secs["col%d_seg%d" % (q, 2*i+1)] = self.col_secs[2 * i + 1]
            i = self.num_segments
            secs["col%d_seg%d" % (q, 2*i)] = self.col_secs[2 * i]
            for i in range(self.num_segments):
                secs["diag%d_lowa%d" % (q, i)] = self.diag_secs[2 * i]
                secs["diag%d_lowb%d" % (q, i)] = self.diag_secs[2 * i]
                secs["diag%d_upa%d" % (q, i)] = self.diag_secs[2 * i + 1]
                secs["diag%d_upb%d" % (q, i)] = self.diag_secs[2 * i + 1]
                secs["hor%d_a%d" % (q, i)] = self.hor_secs[i]
                secs["hor%d_b%d" % (q, i)] = self.hor_secs[i]
                secs["diaph%d_%d" % (q, i)] = self.diaph_secs[i]
            i = self.num_segments
            secs["hor%d_a%d" % (q, i)] = self.hor_secs[i]
            secs["hor%d_b%d" % (q, i)] = self.hor_secs[i]
        return secs
    
    def ele2vec(self, elename):
        nodes = self.nodes
        eles = self.elements
        p1 = nodes[eles[elename][0]]
        p2 = nodes[eles[elename][1]]
        return Vec.minus(p2, p1)
    
    def get_axes(self):
        axes = {}
        for q in range(1, 5):
            q_next = q + 1 if q != 4 else 1
            for i in range(self.num_segments):
                axes["diag%d_lowa%d" % (q, i)] = Vec.scale_to(Vec.mul(self.ele2vec("diag%d_lowa%d" % (q, i)), self.ele2vec("col%d_seg0" % (q))), 1.0)
                axes["diag%d_lowb%d" % (q, i)] = Vec.scale_to(Vec.mul(self.ele2vec("diag%d_lowb%d" % (q, i)), self.ele2vec("col%d_seg0" % (q))), 1.0)
                axes["diag%d_upa%d" % (q, i)] = Vec.scale_to(Vec.mul(self.ele2vec("diag%d_upa%d" % (q, i)), self.ele2vec("col%d_seg0" % (q))), 1.0)
                axes["diag%d_upb%d" % (q, i)] = Vec.scale_to(Vec.mul(self.ele2vec("diag%d_upb%d" % (q, i)), self.ele2vec("col%d_seg0" % (q))), 1.0)
                axes["hor%d_a%d" % (q, i)] = Vec.scale_to(Vec.mul(self.ele2vec("hor%d_a%d" % (q, i)), self.ele2vec("col%d_seg0" % (q))), 1.0)
                axes["hor%d_b%d" % (q, i)] = Vec.scale_to(Vec.mul(self.ele2vec("hor%d_b%d" % (q, i)), self.ele2vec("col%d_seg0" % (q))), 1.0)
                axes["col%d_seg%d" % (q, 2*i)] = (0, 0, 1)
                axes["col%d_seg%d" % (q, 2*i+1)] = (0, 0, 1)
                axes["diaph%d_%d" % (q, i)] = (0, 0, 1)
        return axes

    def get_topnsegs(self, col_offset=0, diag_offset=0, nsegs=None, diag_hor_offset=0):
        if nsegs is None:
            nsegs = self.num_segments
        nodes = self.nodes
        new_nodes = {}
        for q in range(1, 5):
            for i in range(self.num_segments - nsegs, self.num_segments):
                for key in (
                    "col%d_joint%d" % (q, i),
                    "col%d_diaph%d" % (q, i),
                    "cross%d_%d" % (q, i),
                ):
                    new_nodes[key] = nodes[key]
            if nsegs == self.num_segments:
                new_nodes["col%d_base" % q] = nodes["col%d_base" % q]
            else:
                i = self.num_segments - nsegs - 1
                new_nodes["col%d_joint%d" % (q, i)] = nodes["col%d_joint%d" % (q, i)]
        if col_offset != 0:
            i = self.num_segments - 1
            for q in range(1, 5):
                col_vec = Vec.minus(
                    nodes["col%d_diaph%d" % (q, i)], nodes["col%d_joint%d" % (q, i)]
                )
                col_offset_vec = Vec.scale_to(col_vec, col_offset)
                col_coords = Vec.add(nodes["col%d_joint%d" % (q, i)], col_offset_vec)
                new_nodes["col%d_joint%d" % (q, i)] = col_coords
        if diag_offset != 0:
            i = self.num_segments - 1
            for q in range(1, 5):
                q_next = 1 if q == 4 else q + 1
                diaga_vec = Vec.minus(nodes["cross%d_%d" % (q, i)], nodes["col%d_joint%d" % (q, i)])
                diaga_offset_vec = Vec.scale_to(diaga_vec, diag_offset)
                diaga_coords = Vec.add(diaga_offset_vec, nodes["col%d_joint%d" % (q, i)])
                new_nodes["diag%d_a%d" % (q, i)] = diaga_coords
                diagb_vec = Vec.minus(
                    nodes["cross%d_%d" % (q_next, i)], nodes["col%d_joint%d" % (q, i)]
                )
                diagb_offset_vec = Vec.scale_to(diagb_vec, diag_offset)
                diagb_coords = Vec.add(diagb_offset_vec, nodes["col%d_joint%d" % (q, i)])
                new_nodes["diag%d_b%d" % (q, i)] = diagb_coords
        eles = self.elements
        secs = self.sections
        axes = self.axes
        new_eles, new_secs, new_axes = {}, {}, {}
        for q in range(1, 5):
            for i in range(self.num_segments - nsegs, self.num_segments):
                for key in (
                    "col%d_seg%d" % (q, 2*i),
                    "col%d_seg%d" % (q, 2*i+1),
                    "diag%d_lowa%d" % (q, i),
                    "diag%d_lowb%d" % (q, i),
                    "diag%d_upa%d" % (q, i),
                    "diag%d_upb%d" % (q, i),
                    "hor%d_a%d" % (q, i),
                    "hor%d_b%d" % (q, i),
                    "diaph%d_%d" % (q, i),
                ):
                    new_eles[key] = eles[key]
                    new_secs[key] = secs[key]
                    new_axes[key] = axes[key]
        if col_offset != 0 or diag_offset != 0:
            i = self.num_segments - 1
            for q in range(1, 5):
                q_next = 1 if q == 4 else q + 1
                new_eles["diag%d_upa%d" % (q, i)] = ("cross%d_%d" % (q, i), "diag%d_a%d" % (q, i))
                new_eles["diag%d_upb%d" % (q, i)] = ("diag%d_b%d" % (q, i), "cross%d_%d" % (q_next, i))
                new_axes["diag%d_upa%d" % (q, i)] = axes["diag%d_upa%d" % (q, i)]
                new_axes["diag%d_upb%d" % (q, i)] = axes["diag%d_upb%d" % (q, i)]
        if diag_hor_offset != 0:
            assert diag_offset != 0
            i = self.num_segments - 1
            adjusted_diag_a = Vec.minus(new_nodes["diag1_a%d" % i], [0.0, diag_hor_offset, 0.0])
            adjusted_diag_b = Vec.minus(new_nodes['diag1_b%d' % i], [diag_hor_offset, 0.0, 0.0])
            ref_joint = new_nodes["col4_joint%d" % (i-1)]
            adjusted_cross_z = Vec.solve_pt(0, ref_joint[1], ref_joint[2], adjusted_diag_a[1], adjusted_diag_a[2])
            print("Adj cross", ref_joint[1], ref_joint[2], adjusted_diag_a[1], adjusted_diag_a[2])
            adjusted_cross_x = self.ele2inter(adjusted_cross_z)
            adjusted_cross = [adjusted_cross_x, 0.0, adjusted_cross_z]
            assert adjusted_cross_z < new_nodes['col1_joint%d' % i][2], "Top joint is lower than horizontal member."
            for q in range(1, 5):
                new_nodes["diag%d_a%d" % (q, i)] = self.rot_quad(adjusted_diag_a, q)
                new_nodes["diag%d_b%d" % (q, i)] = self.rot_quad(adjusted_diag_b, q)
                new_nodes["cross%d_%d" % (q, i)] = self.rot_quad(adjusted_cross, q)
                new_nodes["col%d_joint%d" % (q, i)] = self.rot_quad([adjusted_cross_x, adjusted_cross_x, adjusted_cross_z], q) 

        return new_nodes, new_eles, new_secs, new_axes
