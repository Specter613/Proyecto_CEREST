"""
Analiza un CSV capturado con capturar_pid_csv.py: grafica consigna vs
valor real, la salida del PID y el desglose P/I/D por separado, y
calcula métricas de sintonía (overshoot, tiempo de asentamiento) sobre
cada escalón de consigna que detecte automáticamente.

Roll y pitch son lazos de ÁNGULO (grados); yaw es un lazo de VELOCIDAD (°/s).

Uso:
    pip install pandas numpy matplotlib --break-system-packages
    python3 graficar_sintonia_pid.py prueba1.csv
    python3 graficar_sintonia_pid.py prueba1.csv --eje yaw
    python3 graficar_sintonia_pid.py prueba1.csv --eje todos
"""

import argparse

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

UNIDADES = {"roll": "°", "pitch": "°", "yaw": "°/s"}


def detectar_escalones(t, sp, umbral=2.0, separacion_min_s=0.3):
    """Encuentra instantes donde la consigna cambia más de 'umbral'."""
    d_sp = np.abs(np.diff(sp))
    idx = np.where(d_sp > umbral)[0] + 1
    if len(idx) == 0:
        return []
    escalones = [idx[0]]
    for i in idx[1:]:
        if t[i] - t[escalones[-1]] > separacion_min_s:
            escalones.append(i)
    return escalones


def metricas_escalon(t, sp, act, i0, ventana_s=1.5):
    """Overshoot y tiempo de asentamiento (banda ±5% del salto) tras un escalón."""
    sp0, sp_final = sp[i0 - 1] if i0 > 0 else sp[i0], sp[i0]
    salto = sp_final - sp0
    if abs(salto) < 1e-3:
        return None

    mask = (t >= t[i0]) & (t <= t[i0] + ventana_s)
    if not np.any(mask):
        return None
    t_v, act_v = t[mask], act[mask]

    if salto > 0:
        overshoot = (np.max(act_v) - sp_final) / salto * 100.0
    else:
        overshoot = (sp_final - np.min(act_v)) / salto * 100.0
    overshoot = max(0.0, overshoot)

    banda = 0.05 * abs(salto)
    dentro = np.abs(act_v - sp_final) <= banda
    t_asent = None
    for k in range(len(dentro)):
        if np.all(dentro[k:]):
            t_asent = t_v[k] - t[i0]
            break

    return {
        "t_escalon": t[i0],
        "salto": salto,
        "overshoot_pct": overshoot,
        "t_asentamiento_s": t_asent,
    }


def panel_eje(fig, gs_top, gs_bottom, t, sp, act, P, I, D, out, nombre, unidad):
    ax1 = fig.add_subplot(gs_top)
    ax1.plot(t, sp, "--", color="black", lw=1.2, label="Consigna")
    ax1.plot(t, act, color="tab:blue", lw=1.3, label="Real")
    ax1.set_ylabel(unidad)
    ax1.set_title(f"{nombre}: consigna vs real ({unidad})", loc="left", fontweight="bold")
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc="upper right", fontsize=8)

    ax2 = fig.add_subplot(gs_bottom, sharex=ax1)
    ax2.plot(t, P, color="tab:orange", lw=1.0, label="P")
    ax2.plot(t, I, color="tab:green", lw=1.0, label="I")
    ax2.plot(t, D, color="tab:red", lw=1.0, label="D")
    ax2.plot(t, out, color="black", lw=1.4, label="Salida total", alpha=0.7)
    ax2.set_ylabel("Salida PID")
    ax2.set_title(f"{nombre}: desglose P / I / D", loc="left", fontweight="bold")
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc="upper right", fontsize=8, ncol=4)
    return ax1, ax2


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", help="CSV generado por capturar_pid_csv.py")
    ap.add_argument("--eje", choices=["roll", "pitch", "yaw", "ambos", "todos"], default="ambos",
                    help="ambos = roll y pitch; todos = roll, pitch y yaw")
    ap.add_argument("--umbral_escalon", type=float, default=2.0,
                    help="cambio mínimo de consigna para contar como escalón (° o °/s)")
    args = ap.parse_args()

    df = pd.read_csv(args.csv)
    t = (df["tick_ms"].values - df["tick_ms"].values[0]) / 1000.0

    if args.eje == "ambos":
        ejes = ["roll", "pitch"]
    elif args.eje == "todos":
        ejes = ["roll", "pitch", "yaw"]
    else:
        ejes = [args.eje]

    fig = plt.figure(figsize=(12, 4.5 * len(ejes) * 2))
    gs = fig.add_gridspec(len(ejes) * 2, 1, hspace=0.4)

    resumen = {}
    for idx, eje in enumerate(ejes):
        sp = df[f"{eje}_sp"].values
        act = df[f"{eje}_act"].values
        P, I, D, out = df[f"{eje}_P"].values, df[f"{eje}_I"].values, df[f"{eje}_D"].values, df[f"{eje}_out"].values

        panel_eje(fig, gs[idx*2], gs[idx*2+1], t, sp, act, P, I, D, out,
                  eje.capitalize(), UNIDADES[eje])

        escalones = detectar_escalones(t, sp, umbral=args.umbral_escalon)
        filas = []
        for i0 in escalones:
            m = metricas_escalon(t, sp, act, i0)
            if m:
                filas.append(m)
        resumen[eje] = filas

    fig.suptitle(f"Sintonía PID — {args.csv}", fontsize=13)
    salida_png = args.csv.rsplit(".", 1)[0] + "_sintonia.png"
    fig.savefig(salida_png, dpi=150)
    print(f"Figura guardada en: {salida_png}")

    print("\n=== Métricas por escalón detectado ===")
    for eje, filas in resumen.items():
        u = UNIDADES[eje]
        print(f"\n-- {eje.upper()} ({len(filas)} escalones >= {args.umbral_escalon}{u}) --")
        if not filas:
            print("  (ninguno detectado; baja --umbral_escalon si esperabas ver alguno)")
            continue
        for f in filas:
            asent = f"{f['t_asentamiento_s']:.2f}s" if f['t_asentamiento_s'] is not None else "no asentó en la ventana"
            print(f"  t={f['t_escalon']:.2f}s  salto={f['salto']:+.1f}{u}  "
                  f"overshoot={f['overshoot_pct']:.1f}%  asentamiento(±5%)={asent}")

        overshoots = [f["overshoot_pct"] for f in filas]
        asents = [f["t_asentamiento_s"] for f in filas if f["t_asentamiento_s"] is not None]
        print(f"  Promedio: overshoot={np.mean(overshoots):.1f}%  "
              f"asentamiento={np.mean(asents):.2f}s" if asents else "  Promedio: sin asentamientos completos en ventana")

    print("\nGuía rápida de lectura:")
    print("  - Overshoot alto y sostenido -> baja kp, o sube kd")
    print("  - Asentamiento lento pero sin overshoot -> sube kp")
    print("  - Oscila alrededor de la consigna sin asentar -> baja kd (puede estar amplificando ruido)")
    print("  - Error residual pequeño que nunca llega a 0 -> sube ki (con cuidado del anti-windup)")

    plt.show()


if __name__ == "__main__":
    main()

## python3 graficar_sintonia_pid.py prueba1.csv --eje yaw
## python3 graficar_sintonia_pid.py prueba1.csv --eje todos