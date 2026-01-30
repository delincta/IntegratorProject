import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, FancyArrow

def draw_mpc_simulator_block():
    fig, ax = plt.subplots(figsize=(10, 5))

    # Disable axes
    ax.axis('off')

    # MPC block
    mpc = Rectangle((0.1, 0.35), 0.3, 0.3,
                    linewidth=1.5, edgecolor='black', facecolor='none')
    ax.add_patch(mpc)
    ax.text(0.25, 0.5,
            "MPC Controller\n(Optimization)",
            ha='center', va='center', fontsize=12)

    # Simulator block
    sim = Rectangle((0.6, 0.35), 0.3, 0.3,
                    linewidth=1.5, edgecolor='black', facecolor='none')
    ax.add_patch(sim)
    ax.text(0.75, 0.5,
            "Traffic Simulator\n(Nonlinear Network)",
            ha='center', va='center', fontsize=12)

    # Arrow MPC -> Simulator
    ax.annotate("Control inputs\n$u(k), \\, \\Gamma(k)$",
                xy=(0.6, 0.5), xytext=(0.4, 0.5),
                arrowprops=dict(arrowstyle="->", linewidth=1.5),
                ha='center', va='center', fontsize=11)

    # Arrow Simulator -> MPC (feedback)
    ax.annotate("System states\n$x(k+1)$",
                xy=(0.4, 0.32), xytext=(0.6, 0.32),
                arrowprops=dict(arrowstyle="->", linewidth=1.5),
                ha='center', va='center', fontsize=11)

    # Global title
    ax.set_title("Closed-Loop MPC–Simulator Architecture", fontsize=14)

    plt.tight_layout()
    plt.show()


# Call the function
draw_mpc_simulator_block()
