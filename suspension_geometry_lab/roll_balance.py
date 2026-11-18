"""Quasi-static load transfer and an analytical rear RC solution.

Treat all mass as sprung, with a rigid chassis, no aero, no unsprung-mass
moment or tire load-sensitivity model. Only the zero-travel pose is scored.
"""
from dataclasses import dataclass
import math

from .project import static_objects


@dataclass(frozen=True)
class LoadTransferBalance:
    front_weight_fraction: float
    front_transfer_fraction: float
    front_transfer_per_g: float
    rear_transfer_per_g: float
    front_rc_m: float
    rear_rc_m: float
    cg_height_m: float

    @property
    def error(self):
        """Signed difference from equal front/rear load transfer (50% front)."""
        return self.front_transfer_fraction-.5


@dataclass(frozen=True)
class AnalyticRearRC:
    front_rc_m: float
    rear_rc_m: float
    rear_design_m: float
    cg_height_m: float


def _balance_inputs(vehicle, cg_height_m):
    keys = ("wheelbase", "cg_to_front", "track_front", "track_rear", "roll_stiffness_ratio")
    values = [vehicle[key] for key in keys]
    cg = vehicle["cg_height"] if cg_height_m is None else cg_height_m
    if any(type(value) not in (int, float) or not math.isfinite(value) for value in [*values, cg]):
        raise ValueError("載荷轉移平衡輸入必須是有限數值。")
    wheelbase, distance, front_track, rear_track, ratio = values
    if not (0 < distance < wheelbase and min(front_track, rear_track, ratio, cg) > 0):
        raise ValueError("輪距、側傾剛性比與重心高度須為正值，且重心須位於軸距內。")
    wr = distance/wheelbase
    return 1-wr, wr, front_track, rear_track, ratio/(1+ratio), 1/(1+ratio), cg


def load_transfer_balance(vehicle, front_rc_m, rear_rc_m, *, cg_height_m=None):
    """Calculate axle transfer shares using moment balance.

    ΔFz_i is the load transferred from inside to outside (half the difference
    of wheel loads). With w_i the axle weight fraction, s_i its roll-stiffness
    fraction and H = hCG - (wF*hRCF + wR*hRCR):

        ΔFz_i / (m*ay) = (w_i*hRC_i + s_i*H) / track_i

    Total elastic moment is shared by *roll* stiffness, so track is not
    squared again. Equal transfer is a 50/50 target even for unequal weights;
    it is not a prediction of neutral steering or actual understeer gradient.
    """
    wf, wr, front_track, rear_track, sf, sr, cg = _balance_inputs(vehicle, cg_height_m)
    if any(type(value) not in (int, float) or not math.isfinite(value)
           for value in (front_rc_m, rear_rc_m)):
        raise ValueError("載荷轉移平衡輸入必須是有限數值。")
    lever = cg-wf*front_rc_m-wr*rear_rc_m
    front = (wf*front_rc_m+sf*lever)/front_track
    rear = (wr*rear_rc_m+sr*lever)/rear_track
    if not (math.isfinite(front) and math.isfinite(rear) and min(front, rear) >= 0 and front+rear > 0):
        raise ValueError("此幾何無法定義有效的前後向外載荷轉移占比。")
    return LoadTransferBalance(wf, front/(front+rear), front/wf, rear/wr,
                               front_rc_m, rear_rc_m, cg)


def solve_rear_roll_center(vehicle, front_rc_m, *, cg_height_m=None):
    """Closed form from how_to_find_rear_roll_center.md: ΔFz_F == ΔFz_R.

    Divide all lateral forces by their common m*ay factor: A=wF/tF,
    B=wR/tR, C=(1-lambdaK)/tF-lambdaK/tR, lambdaK=1/(1+KphiF/KphiR).
    The denominator B+C*wR is evaluated in its equivalent positive form to
    avoid subtractive cancellation when the front roll stiffness is small.
    """
    wf, wr, tf, tr, sf, sr, cg = _balance_inputs(vehicle, cg_height_m)
    if type(front_rc_m) not in (int, float) or not math.isfinite(front_rc_m):
        raise ValueError("前軸 RC 高度必須是有限數值。")
    a, c = wf/tf, sf/tf-sr/tr
    denominator = wr*sf*(1/tf+1/tr)  # B+C*wR
    if denominator <= 0 or not math.isfinite(denominator):
        raise ValueError("解析式分母無效，無法求得唯一的有限後軸 RC。")
    height = (a*front_rc_m+c*(cg-wf*front_rc_m))/denominator
    if not math.isfinite(height):
        raise ValueError("解析式未得到有限的後軸 RC 高度。")
    return height


def analytic_rear_solution(project):
    """Solve physical RC height, then invert installation camber analytically.

    Mechanism construction stays at camber=0. For a right contact (x,z),
    IC=(track/2-FVSA, FVSA/(track/2)*design_height). Its mirrored force-line
    intersection gives actual_height=x*(IC_z-z)/(x-IC_x). Inverting this
    affine map converts the physical target into the stored design height.
    No Engine builds, numerical iteration or height scan is needed here.
    """
    car, _, _ = static_objects(project)
    vehicle = project["vehicle"]
    wf, wr = map(float, car.cg_rate)
    contacts = {}
    for axle, track, radius in (("front", car.tf, car.load_Radius[0]),
                                ("rear", car.tr, car.load_Radius[1])):
        gamma = math.radians(project[axle]["static_camber_deg"])
        x = float(track/2-radius*math.sin(gamma))
        z = float(radius*(1-math.cos(gamma)))
        ic_x = track/2-project[axle]["fvsa"]
        if x <= 1e-12 or abs(x-ic_x) <= 1e-12:
            raise ValueError(f"{axle}：靜態接地點與瞬心無法定義有效的 RC 解析換算。")
        contacts[axle] = (x, z, ic_x)
    fx, fz, fic_x = contacts["front"]
    front_rc = fx*(project["front"]["fvsa"]/(car.tf/2)*project["front"]["rc_height"]-fz)/(fx-fic_x)
    cg = float(car.h_cog-wf*fz-wr*contacts["rear"][1])
    rear_rc = solve_rear_roll_center(vehicle, float(front_rc), cg_height_m=cg)
    rx, rz, ric_x = contacts["rear"]
    design = car.tr/2/project["rear"]["fvsa"]*(rz+(rx-ric_x)/rx*rear_rc)
    if not math.isfinite(design):
        raise ValueError("靜態 Camber 換算未得到有限的後軸 RC 設計高度。")
    return AnalyticRearRC(float(front_rc), rear_rc, float(design), cg)


def balance_from_engine(engine):
    states = [engine.fv[axle].solve(0., 0., gains=False)["metrics"] for axle in ("front", "rear")]
    if any(state["rc_y_prime_mm"] is None for state in states):
        raise ValueError("零行程 RC 無有限交點，無法計算載荷轉移平衡。")
    vehicle = engine.project["vehicle"]
    wr = vehicle["cg_to_front"]/vehicle["wheelbase"]
    # The camber installation can raise either static contact plane slightly.
    # Interpolate ground at CG using the same weights as the RC roll axis.
    cg = ((1-wr)*states[0]["cg_height_mm"]+wr*states[1]["cg_height_mm"])/1000
    return load_transfer_balance(vehicle, states[0]["rc_y_prime_mm"]/1000,
                                 states[1]["rc_y_prime_mm"]/1000, cg_height_m=cg)
