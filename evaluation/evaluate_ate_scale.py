# Modified by Raul Mur-Artal
# Updated for Python 3.10 by ChatGPT
# Automatically compute the optimal scale factor for monocular VO/SLAM.

import sys
import numpy as np
import argparse
import associate


def align(model, data):
    """Align two trajectories using the method of Horn (closed-form)."""

    np.set_printoptions(precision=3, suppress=True)
    model_zerocentered = model - model.mean(1)
    data_zerocentered = data - data.mean(1)

    W = np.zeros((3, 3))
    for column in range(model.shape[1]):
        W += np.outer(model_zerocentered[:, column], data_zerocentered[:, column])

    U, d, Vh = np.linalg.svd(W.T)
    S = np.identity(3)
    if np.linalg.det(U) * np.linalg.det(Vh) < 0:
        S[2, 2] = -1
    rot = U @ S @ Vh

    rotmodel = rot @ model_zerocentered
    dots = 0.0
    norms = 0.0

    for column in range(data_zerocentered.shape[1]):
        dots += np.dot(data_zerocentered[:, column].T, rotmodel[:, column])
        normi = np.linalg.norm(model_zerocentered[:, column])
        norms += normi * normi

    s = float(dots / norms)

    transGT = data.mean(1) - s * rot @ model.mean(1)
    trans = data.mean(1) - rot @ model.mean(1)

    model_alignedGT = s * rot @ model + transGT
    model_aligned = rot @ model + trans

    alignment_errorGT = model_alignedGT - data
    alignment_error = model_aligned - data

    trans_errorGT = np.sqrt(np.sum(np.multiply(alignment_errorGT, alignment_errorGT), 0)).A1
    trans_error = np.sqrt(np.sum(np.multiply(alignment_error, alignment_error), 0)).A1

    return rot, transGT, trans_errorGT, trans, trans_error, s


def plot_traj(ax, stamps, traj, style, color, label):
    """Plot a trajectory using matplotlib."""

    stamps = sorted(stamps)
    interval = np.median([s - t for s, t in zip(stamps[1:], stamps[:-1])])
    x, y = [], []
    last = stamps[0]
    for i in range(len(stamps)):
        if stamps[i] - last < 2 * interval:
            x.append(traj[i][0])
            y.append(traj[i][1])
        elif len(x) > 0:
            ax.plot(x, y, style, color=color, label=label)
            label = ""
            x, y = [], []
        last = stamps[i]
    if len(x) > 0:
        ax.plot(x, y, style, color=color, label=label)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Compute absolute trajectory error (ATE).")
    parser.add_argument("first_file", help="ground truth trajectory (timestamp tx ty tz qx qy qz qw)")
    parser.add_argument("second_file", help="estimated trajectory (timestamp tx ty tz qx qy qz qw)")
    parser.add_argument("--offset", type=float, default=0.0, help="time offset for second trajectory (s)")
    parser.add_argument("--scale", type=float, default=1.0, help="scale factor for second trajectory")
    parser.add_argument("--max_difference", type=float, default=0.02,
                        help="max allowed time diff for matching entries (s)")
    parser.add_argument("--save", help="save aligned trajectory (stamp x y z)")
    parser.add_argument("--save_associations", help="save associations (stamp1 x1 y1 z1 stamp2 x2 y2 z2)")
    parser.add_argument("--plot", help="plot trajectories to an image (e.g. out.pdf)")
    parser.add_argument("--verbose", action="store_true", help="print full statistics")
    parser.add_argument("--verbose2", action="store_true", help="print with/without scale correction")
    args = parser.parse_args()

    first_list = associate.read_file_list(args.first_file, False)
    second_list = associate.read_file_list(args.second_file, False)

    matches = associate.associate(first_list, second_list, args.offset, args.max_difference)
    if len(matches) < 2:
        sys.exit("Couldn't find matching timestamp pairs between groundtruth and estimated trajectory!")

    first_xyz = np.matrix([[float(value) for value in first_list[a][0:3]] for a, b in matches]).T
    second_xyz = np.matrix([[float(value) * args.scale for value in second_list[b][0:3]] for a, b in matches]).T

    rot, transGT, trans_errorGT, trans, trans_error, scale = align(second_xyz, first_xyz)
    second_xyz_aligned = scale * rot @ second_xyz + trans

    first_stamps = sorted(first_list.keys())
    first_xyz_full = np.matrix([[float(value) for value in first_list[b][0:3]] for b in first_stamps]).T

    second_stamps = sorted(second_list.keys())
    second_xyz_full = np.matrix([[float(value) * args.scale for value in second_list[b][0:3]] for b in second_stamps]).T
    second_xyz_full_aligned = scale * rot @ second_xyz_full + trans

    if args.verbose:
        print(f"compared_pose_pairs {len(trans_error)} pairs")
        print(f"absolute_translational_error.rmse {np.sqrt(np.dot(trans_error, trans_error) / len(trans_error)):.6f} m")
        print(f"absolute_translational_error.mean {np.mean(trans_error):.6f} m")
        print(f"absolute_translational_error.median {np.median(trans_error):.6f} m")
        print(f"absolute_translational_error.std {np.std(trans_error):.6f} m")
        print(f"absolute_translational_error.min {np.min(trans_error):.6f} m")
        print(f"absolute_translational_error.max {np.max(trans_error):.6f} m")
        print(f"max idx: {np.argmax(trans_error)}")
    else:
        print(f"{np.sqrt(np.dot(trans_error, trans_error) / len(trans_error)):.6f},"
              f"{scale:.6f},"
              f"{np.sqrt(np.dot(trans_errorGT, trans_errorGT) / len(trans_errorGT)):.6f}")

    if args.verbose2:
        print(f"compared_pose_pairs {len(trans_error)} pairs")
        print(f"absolute_translational_error.rmse {np.sqrt(np.dot(trans_error, trans_error) / len(trans_error)):.6f} m")
        print(f"absolute_translational_errorGT.rmse {np.sqrt(np.dot(trans_errorGT, trans_errorGT) / len(trans_errorGT)):.6f} m")

    if args.save_associations:
        with open(args.save_associations, "w") as f:
            for (a, b), (x1, y1, z1), (x2, y2, z2) in zip(
                matches, first_xyz.T.A, second_xyz_aligned.T.A
            ):
                f.write(f"{a} {x1} {y1} {z1} {b} {x2} {y2} {z2}\n")

    if args.save:
        with open(args.save, "w") as f:
            for stamp, line in zip(second_stamps, (second_xyz_full_aligned.T.A)):
                f.write(f"{stamp} " + " ".join(f"{d}" for d in line) + "\n")

    if args.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig = plt.figure()
        ax = fig.add_subplot(111)
        plot_traj(ax, first_stamps, first_xyz_full.T.A, "-", "black", "ground truth")
        plot_traj(ax, second_stamps, second_xyz_full_aligned.T.A, "-", "blue", "estimated")

        # label = "difference"
        # for (a, b), (x1, y1, z1), (x2, y2, z2) in zip(matches, first_xyz.T.A, second_xyz_aligned.T.A):
        #     ax.plot([x1, x2], [y1, y2], "-", color="red", label=label)
        #     label = ""

        ax.legend()
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        plt.axis("equal")
        plt.savefig(args.plot, format="pdf")
