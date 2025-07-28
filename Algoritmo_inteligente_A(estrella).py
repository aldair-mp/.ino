# ---- LIBRERIAS EN PYTHON ----
import socket      # Comunicación Wi-Fi (UDP)
import threading   # Hilos para ejecución en paralelo
import matplotlib.pyplot as plt  # Dibujar gráficos (mapa, ruta)
import numpy as np              # Manejo eficiente de matrices
import heapq        # Cola de prioridad para A*
import tkinter as tk  # Interfaz gráfica (ventana, etiquetas)
import time          # Medir tiempos y temporizadores

# ---- CONFIGURACIÓN ----
MAZESIZE_X = 16
MAZESIZE_Y = 16
UDP_PORT = 12345

mtx = 2
mty = 2

mapa_paredes = [[{'N': 0, 'E': 0, 'S': 0, 'W': 0} for _ in range(MAZESIZE_X)] for _ in range(MAZESIZE_Y)]
steps_map = np.full((MAZESIZE_Y, MAZESIZE_X), -1)
modo = "walls"
fila_actual = 0

# ---- INTERFAZ GRÁFICA ----
root = tk.Tk()
root.title("📡 Sensores y Mapa")
root.geometry("300x220")  # Ventana más pequeña

sensor_frame = tk.Frame(root, width=200)
sensor_frame.pack(fill="both", expand=True)

sensor_labels = {}
for name in ["R", "FR", "FL", "L", "V"]:
    lbl = tk.Label(sensor_frame, text=f"{name}: 0", font=("Arial", 12))
    lbl.pack(anchor="w", padx=10, pady=2)
    sensor_labels[name] = lbl

# Estado de conexión
status_label = tk.Label(sensor_frame, text="❌ Estado: Desconectado", font=("Arial", 12), fg="red")
status_label.pack(anchor="w", padx=10, pady=5)

# ---- SOCKET UDP ----
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("", UDP_PORT))
sock.settimeout(1.0)

# ---- CONEXIÓN ----
last_sensor_time = 0  # Tiempo de último dato de sensor recibido


def check_connection():
    global last_sensor_time
    now = time.time()
    if now - last_sensor_time < 2:  # 2 segundos sin datos = desconectado
        status_label.config(text="✅ Estado: Conectado", fg="green")
    else:
        status_label.config(text="❌ Estado: Desconectado", fg="red")
    root.after(1000, check_connection)


# ---- DIBUJAR MAPA ----
def draw_map():
    fig, ax = plt.subplots(figsize=(6, 6))
    for y in range(MAZESIZE_Y):
        for x in range(MAZESIZE_X):
            p = mapa_paredes[y][x]
            if p['N']:
                ax.plot([x, x + 1], [y + 1, y + 1], 'k', linewidth=2)
            if p['E']:
                ax.plot([x + 1, x + 1], [y, y + 1], 'k', linewidth=2)
            if p['S']:
                ax.plot([x, x + 1], [y, y], 'k', linewidth=2)
            if p['W']:
                ax.plot([x, x], [y, y + 1], 'k', linewidth=2)
            ax.text(x + 0.5, y + 0.5, f"({x},{y})", ha='center', va='center', fontsize=6, color='gray')

    ax.plot([0, MAZESIZE_X], [0, 0], 'k', linewidth=4)
    ax.plot([0, MAZESIZE_X], [MAZESIZE_Y, MAZESIZE_Y], 'k', linewidth=4)
    ax.plot([0, 0], [0, MAZESIZE_Y], 'k', linewidth=4)
    ax.plot([MAZESIZE_X, MAZESIZE_X], [0, MAZESIZE_Y], 'k', linewidth=4)

    ax.add_patch(plt.Rectangle((0, 0), 1, 1, color='red', alpha=0.4))
    ax.add_patch(plt.Rectangle((mtx, mty), 1, 1, color='lime', alpha=0.4))

    def vecinos(x, y):
        moves = []
        if not mapa_paredes[y][x]['N'] and y < MAZESIZE_Y - 1:
            moves.append((x, y + 1))
        if not mapa_paredes[y][x]['E'] and x < MAZESIZE_X - 1:
            moves.append((x + 1, y))
        if not mapa_paredes[y][x]['S'] and y > 0:
            moves.append((x, y - 1))
        if not mapa_paredes[y][x]['W'] and x > 0:
            moves.append((x - 1, y))
        return moves

    def heuristica(a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def a_estrella(inicio, meta):
        open_set = []
        heapq.heappush(open_set, (0, inicio))
        came_from = {}
        g_score = {inicio: 0}

        while open_set:
            _, actual = heapq.heappop(open_set)
            if actual == meta:
                camino = [actual]
                while actual in came_from:
                    actual = came_from[actual]
                    camino.append(actual)
                camino.reverse()
                return camino
            for vecino in vecinos(*actual):
                tentative_g = g_score[actual] + 1
                if vecino not in g_score or tentative_g < g_score[vecino]:
                    came_from[vecino] = actual
                    g_score[vecino] = tentative_g
                    f = tentative_g + heuristica(vecino, meta)
                    heapq.heappush(open_set, (f, vecino))
        return None

    ruta = a_estrella((0, 0), (mtx, mty))
    if ruta:
        xs, ys = zip(*[(x + 0.5, y + 0.5) for (x, y) in ruta])
        ax.plot(xs, ys, color='blue', linewidth=2)

    ax.set_aspect('equal')
    ax.set_xlim(0, MAZESIZE_X)
    ax.set_ylim(0, MAZESIZE_Y)
    ax.grid(True, which='both', color='lightgray', linewidth=0.5)
    ax.set_title("📏 Mapa con Ruta Más Corta (A*)")
    plt.show()

# ---- ESCUCHAR DATOS UDP ----
def listen_udp():
    global modo, fila_actual, last_sensor_time
    while True:
        try:
            data, _ = sock.recvfrom(1024)
            msg = data.decode().strip()

            if any(label in msg for label in ["R:", "FR:", "FL:", "L:", "V:"]):
                partes = msg.replace("mV", "").split()
                valores = {p.split(":")[0]: p.split(":")[1] for p in partes if ":" in p}
                for clave, valor in valores.items():
                    if clave in sensor_labels:
                        sensor_labels[clave].config(text=f"{clave}: {valor}")
                last_sensor_time = time.time()  # Actualiza tiempo al recibir sensores

            elif msg == "END":
                print("✅ Mapa de paredes recibido.")
                modo = "steps"
                fila_actual = 0
            elif msg == "ENDSTEP":
                print("✅ Mapa de pasos recibido.")
                draw_map()
            else:
                celdas = msg.split()

                if modo == "walls":
                    if fila_actual >= MAZESIZE_Y:
                        continue
                    es_valido = all(len(c) == 4 and all(d in "01" for d in c) for c in celdas)
                    if not es_valido:
                        continue
                    for x, celda in enumerate(celdas):
                        mapa_paredes[MAZESIZE_Y - 1 - fila_actual][x] = {
                            'N': int(celda[0]),
                            'E': int(celda[1]),
                            'S': int(celda[2]),
                            'W': int(celda[3])
                        }
                elif modo == "steps":
                    if fila_actual >= MAZESIZE_Y:
                        continue
                    for x, valor in enumerate(celdas):
                        try:
                            steps_map[MAZESIZE_Y - 1 - fila_actual][x] = int(valor)
                        except:
                            pass
                fila_actual += 1

        except socket.timeout:
            continue

# ---- INICIO ----
threading.Thread(target=listen_udp, daemon=True).start()
check_connection()  # Inicia monitoreo de conexión
root.mainloop()
