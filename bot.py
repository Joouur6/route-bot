#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Bot de Telegram para optimización de rutas de reparto.
VERSIÓN 2 - Mejor manejo de errores en geocodificación.
"""

import os
import re
import sys
import time
import logging
import requests
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
from datetime import datetime
from dotenv import load_dotenv

from telegram import Update, InlineKeyboardButton, InlineKeyboardMarkup
from telegram.ext import (
    Application, CommandHandler, ConversationHandler,
    MessageHandler, ContextTypes, filters,
)
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

# ─── Configuración ───
load_dotenv()
TOKEN = os.getenv("BOT_TOKEN")
ORS_API_KEY = os.getenv("ORS_API_KEY")
DEPOT_ADDRESS = os.getenv("DEPOT_ADDRESS", "Madrid, España")
START_HOUR = os.getenv("START_HOUR", "09:30")
END_HOUR = os.getenv("END_HOUR", "15:00")
PORT = int(os.getenv("PORT", "8000"))

(ESPERANDO_DIRECCIONES, ESPERANDO_ORIGEN) = range(2)

logging.basicConfig(
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    level=logging.INFO,
)
logger = logging.getLogger(__name__)

# ─── Servidor HTTP para Render.com ───
class HealthHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        self.send_response(200)
        self.send_header("Content-type", "text/plain")
        self.end_headers()
        self.wfile.write(b"Bot is running")
    def log_message(self, format, *args):
        pass

def start_http_server():
    server = HTTPServer(("0.0.0.0", PORT), HealthHandler)
    logger.info(f"Health server on port {PORT}")
    server.serve_forever()

# ─── Geocodificación con timeout y retry ───

def nominatim_geocode(address: str, max_retries=2):
    url = "https://nominatim.openstreetmap.org/search"
    params = {
        "q": address,
        "format": "json",
        "limit": 1,
        "addressdetails": 1,
        "countrycodes": "es",  # Limitar a España
    }
    headers = {"User-Agent": "RouteOptimizerBot/1.0"}

    for attempt in range(max_retries):
        try:
            time.sleep(1.2)  # Respetar rate limit
            r = requests.get(url, params=params, headers=headers, timeout=10)
            data = r.json()
            if data:
                return {
                    "lat": float(data[0]["lat"]),
                    "lon": float(data[0]["lon"]),
                    "display_name": data[0]["display_name"],
                }
            logger.warning(f"Nominatim no encontró: {address}")
            return None
        except requests.exceptions.Timeout:
            logger.warning(f"Timeout en intento {attempt+1} para: {address}")
            if attempt < max_retries - 1:
                time.sleep(2)
        except Exception as e:
            logger.error(f"Error geocodificando '{address}': {e}")
            return None
    return None


def ors_matrix(coordinates):
    if not ORS_API_KEY:
        logger.error("Falta ORS_API_KEY")
        return None, None
    url = "https://api.openrouteservice.org/v2/matrix/driving-car"
    headers = {"Authorization": ORS_API_KEY, "Content-Type": "application/json"}
    body = {"locations": coordinates, "metrics": ["duration", "distance"], "units": "m"}
    try:
        r = requests.post(url, json=body, headers=headers, timeout=30)
        r.raise_for_status()
        data = r.json()
        return data["durations"], data["distances"]
    except Exception as e:
        logger.error(f"Error ORS Matrix: {e}")
        return None, None


def parse_input(text: str):
    lines = [l.strip() for l in text.strip().split("\n") if l.strip() and not l.strip().startswith("/")]
    deliveries = []
    start_dt = datetime.strptime(START_HOUR, "%H:%M")
    end_dt = datetime.strptime(END_HOUR, "%H:%M")
    global_tw_min = (start_dt.hour * 60 + start_dt.minute)
    global_tw_max = (end_dt.hour * 60 + end_dt.minute)

    for line in lines:
        # Ignorar líneas vacías o comandos
        if not line or line.startswith("/"):
            continue

        parts = line.split("|")
        address = parts[0].strip()
        has_tw = False
        tw_min = global_tw_min
        tw_max = global_tw_max

        if len(parts) > 1:
            time_str = parts[1].strip()
            match = re.match(r"(\d{1,2}):?(\d{2})", time_str)
            if match:
                h, m = int(match.group(1)), int(match.group(2))
                fixed_min = h * 60 + m
                margin = 15
                tw_min = max(global_tw_min, fixed_min - margin)
                tw_max = min(global_tw_max, fixed_min + margin)
                has_tw = True

        deliveries.append({
            "address": address,
            "tw_min": tw_min,
            "tw_max": tw_max,
            "has_tw": has_tw,
        })
    return deliveries, global_tw_min, global_tw_max


def solve_tsptw(deliveries, duration_matrix, distance_matrix, global_tw_min, global_tw_max):
    n = len(deliveries)
    if n == 0:
        return None, None, None
    if n == 1:
        return [{"node": 0, "arrival_minutes": global_tw_min}], 0, 0

    manager = pywrapcp.RoutingIndexManager(n, 1, 0)
    routing = pywrapcp.RoutingModel(manager)

    def transit_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        dur = duration_matrix[from_node][to_node]
        dist = distance_matrix[from_node][to_node]
        return int(dur * 0.6 + (dist / 1000.0) * 0.4 * 60)

    transit_cb_index = routing.RegisterTransitCallback(transit_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_cb_index)

    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return int(duration_matrix[from_node][to_node])

    time_cb_index = routing.RegisterTransitCallback(time_callback)
    horizon_seconds = (global_tw_max - global_tw_min) * 60
    routing.AddDimension(time_cb_index, 60, horizon_seconds + 3600, False, "Time")
    time_dimension = routing.GetDimensionOrDie("Time")

    for i, deliv in enumerate(deliveries):
        index = manager.NodeToIndex(i)
        tw_start = (deliv["tw_min"] - global_tw_min) * 60
        tw_end = (deliv["tw_max"] - global_tw_min) * 60
        time_dimension.CumulVar(index).SetRange(tw_start, tw_end)

    routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.End(0)))

    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    search_parameters.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    search_parameters.time_limit.FromSeconds(10)

    solution = routing.SolveWithParameters(search_parameters)
    if not solution:
        return None, None, None

    route = []
    index = routing.Start(0)
    while not routing.IsEnd(index):
        node = manager.IndexToNode(index)
        time_var = time_dimension.CumulVar(index)
        arrival_seconds = solution.Min(time_var)
        arrival_minutes_global = global_tw_min + (arrival_seconds // 60)
        route.append({"node": node, "arrival_minutes": arrival_minutes_global})
        index = solution.Value(routing.NextVar(index))
    node = manager.IndexToNode(index)
    time_var = time_dimension.CumulVar(index)
    arrival_seconds = solution.Min(time_var)
    arrival_minutes_global = global_tw_min + (arrival_seconds // 60)
    route.append({"node": node, "arrival_minutes": arrival_minutes_global})

    total_distance = 0
    total_duration = 0
    for i in range(len(route) - 1):
        a = route[i]["node"]
        b = route[i+1]["node"]
        total_distance += distance_matrix[a][b]
        total_duration += duration_matrix[a][b]

    return route, total_distance, total_duration


def minutes_to_hour_str(minutes):
    h = minutes // 60
    m = minutes % 60
    return f"{h:02d}:{m:02d}"


# ─── Handlers Telegram ───

async def start(update: Update, context: ContextTypes.DEFAULT_TYPE):
    text = (
        "👋 ¡Hola! Soy tu optimizador de rutas.\n\n"
        "Usa /ruta para empezar.\n"
        "Si alguna tiene hora fija, escríbela así:\n"
        "<code>Calle Mayor 5, Madrid | 11:30</code>\n\n"
        "Te devolveré el orden óptimo y enlaces de Waze."
    )
    await update.message.reply_html(text)


async def ruta_command(update: Update, context: ContextTypes.DEFAULT_TYPE):
    context.user_data.clear()
    await update.message.reply_text(
        "📍 Envíame las direcciones de hoy, una por línea.\n"
        "Si tiene hora fija, añade ' | HH:MM' al final.\n\n"
        "Ejemplo:\n"
        "Calle Alcalá 45, Madrid\n"
        "Av. del Puerto 12, Madrid | 11:30\n"
        "Plaza España 3, Madrid\n\n"
        "Cuando termines, escribe /listo"
    )
    return ESPERANDO_DIRECCIONES


async def recibir_direcciones(update: Update, context: ContextTypes.DEFAULT_TYPE):
    text = update.message.text.strip()

    if text.lower() == "/listo":
        if not context.user_data.get("raw_lines"):
            await update.message.reply_text("❌ No has enviado direcciones todavía. Escribe al menos una.")
            return ESPERANDO_DIRECCIONES

        raw = "\n".join(context.user_data["raw_lines"])
        deliveries, global_tw_min, global_tw_max = parse_input(raw)

        if not deliveries:
            await update.message.reply_text("❌ No pude interpretar ninguna dirección válida.")
            return ESPERANDO_DIRECCIONES

        context.user_data["deliveries"] = deliveries
        context.user_data["global_tw_min"] = global_tw_min
        context.user_data["global_tw_max"] = global_tw_max

        await update.message.reply_text(
            f"✅ He recibido {len(deliveries)} direcciones.\n"
            f"¿Desde qué dirección sales? (Escribe /omitir para usar: {DEPOT_ADDRESS})"
        )
        return ESPERANDO_ORIGEN

    if text.startswith("/"):
        await update.message.reply_text("❓ Comando no reconocido. Sigue enviando direcciones o escribe /listo para procesar.")
        return ESPERANDO_DIRECCIONES

    if "raw_lines" not in context.user_data:
        context.user_data["raw_lines"] = []
    context.user_data["raw_lines"].append(text)
    count = len(context.user_data["raw_lines"])
    await update.message.reply_text(f"📥 Dirección {count} guardada. Sigue o escribe /listo.")
    return ESPERANDO_DIRECCIONES


async def recibir_origen(update: Update, context: ContextTypes.DEFAULT_TYPE):
    text = update.message.text.strip()
    if text.lower() != "/omitir":
        context.user_data["origin_address"] = text
    else:
        context.user_data["origin_address"] = DEPOT_ADDRESS

    await update.message.reply_text("🧠 Procesando ruta óptima... esto puede tardar 1-2 minutos.")

    deliveries = context.user_data["deliveries"]
    origin_address = context.user_data["origin_address"]

    # Geocodificar origen
    await update.message.reply_text("📍 Geocodificando dirección de origen...")
    origin_geo = nominatim_geocode(origin_address)
    if not origin_geo:
        await update.message.reply_text(f"❌ No pude geocodificar el origen: {origin_address}. Prueba con una dirección más completa (incluyendo ciudad).")
        return ConversationHandler.END

    # Geocodificar entregas
    await update.message.reply_text(f"📍 Geocodificando {len(deliveries)} direcciones...")
    coords = [[origin_geo["lon"], origin_geo["lat"]]]
    display_names = [origin_geo["display_name"]]
    failed = []
    valid_deliveries = []

    for i, d in enumerate(deliveries):
        await update.message.reply_text(f"🔍 Buscando dirección {i+1}/{len(deliveries)}: {d['address'][:40]}...")
        geo = nominatim_geocode(d["address"])
        if geo:
            coords.append([geo["lon"], geo["lat"]])
            display_names.append(geo["display_name"])
            valid_deliveries.append(d)
        else:
            failed.append(d["address"])
            logger.warning(f"No se pudo geocodificar: {d['address']}")

    if failed:
        failed_text = "\n".join([f"• {f}" for f in failed])
        await update.message.reply_text(
            f"⚠️ No pude geocodificar estas direcciones (se omiten):\n{failed_text}\n\n"
            f"Consejo: Añade la ciudad y código postal para mejorar la precisión."
        )

    if not valid_deliveries:
        await update.message.reply_text("❌ Ninguna dirección pudo ser geocodificada. Prueba con direcciones más completas.")
        return ConversationHandler.END

    # Consultar ORS
    await update.message.reply_text("🛣️ Consultando rutas reales por carretera...")
    durations, distances = ors_matrix(coords)
    if durations is None:
        await update.message.reply_text("❌ Error al consultar OpenRouteService. Revisa tu API key.")
        return ConversationHandler.END

    # Resolver TSP
    await update.message.reply_text("🧮 Calculando ruta óptima...")
    depot_entry = {
        "address": origin_address,
        "tw_min": context.user_data["global_tw_min"],
        "tw_max": context.user_data["global_tw_max"],
        "has_tw": False,
    }
    full_nodes = [depot_entry] + valid_deliveries

    route, total_distance, total_duration = solve_tsptw(
        full_nodes, durations, distances,
        context.user_data["global_tw_min"],
        context.user_data["global_tw_max"],
    )

    if not route:
        await update.message.reply_text(
            "❌ No encontré una ruta válida con esas restricciones de horario.\n"
            "Las horas fijas pueden ser incompatibles con la distancia entre puntos."
        )
        return ConversationHandler.END

    # Construir respuesta
    lines = []
    lines.append("📋 <b>RUTA OPTIMIZADA</b> (ahorro combustible)")
    lines.append(f"⛽ Distancia total: <b>{total_distance/1000:.1f} km</b>")
    lines.append(f"⏱️ Tiempo conducción: <b>{total_duration//60} min</b>")
    if failed:
        lines.append(f"⚠️ Omitidas: {len(failed)} direcciones no encontradas")
    lines.append("")

    keyboard = []

    for i, stop in enumerate(route):
        node_idx = stop["node"]
        if node_idx >= len(display_names):
            continue
        name = display_names[node_idx]
        arr = minutes_to_hour_str(stop["arrival_minutes"])
        lat = coords[node_idx][1]
        lon = coords[node_idx][0]

        if i == 0:
            lines.append(f"🏭 <b>Salida:</b> {name} — <b>{arr}</b>")
        elif i == len(route) - 1:
            lines.append(f"🏁 <b>Regreso:</b> {name} — <b>{arr}</b>")
        else:
            tw_info = ""
            if full_nodes[node_idx]["has_tw"]:
                tw_info = " ⏰ (hora fija)"
            lines.append(f"{i}. 📦 {name} — Llegada: <b>{arr}</b>{tw_info}")
            waze_url = f"https://waze.com/ul?ll={lat},{lon}&navigate=yes"
            keyboard.append([InlineKeyboardButton(f"🚗 Waze → Parada {i}", url=waze_url)])

    if len(route) > 1:
        last = route[-2]
        last_node = last["node"]
        waze_full = f"https://www.waze.com/livemap/directions?from=ll.{coords[0][1]}%2C{coords[0][0]}&to=ll.{coords[last_node][1]}%2C{coords[last_node][0]}"
        keyboard.insert(0, [InlineKeyboardButton("🗺️ Waze: Ruta completa", url=waze_full)])

    reply_markup = InlineKeyboardMarkup(keyboard)

    await update.message.reply_text("\n".join(lines), parse_mode="HTML", reply_markup=reply_markup)
    return ConversationHandler.END


async def cancel(update: Update, context: ContextTypes.DEFAULT_TYPE):
    await update.message.reply_text("❌ Cancelado. Usa /ruta para empezar de nuevo.")
    return ConversationHandler.END


def main():
    if not TOKEN:
        logger.error("Falta BOT_TOKEN"); sys.exit(1)
    if not ORS_API_KEY:
        logger.warning("Falta ORS_API_KEY - las rutas no funcionarán")

    threading.Thread(target=start_http_server, daemon=True).start()

    app = Application.builder().token(TOKEN).build()
    conv_handler = ConversationHandler(
        entry_points=[CommandHandler("ruta", ruta_command)],
        states={
            ESPERANDO_DIRECCIONES: [
                MessageHandler(filters.TEXT & ~filters.COMMAND, recibir_direcciones),
                CommandHandler("listo", recibir_direcciones),
            ],
            ESPERANDO_ORIGEN: [
                MessageHandler(filters.TEXT & ~filters.COMMAND, recibir_origen),
            ],
        },
        fallbacks=[CommandHandler("cancelar", cancel)],
    )
    app.add_handler(CommandHandler("start", start))
    app.add_handler(conv_handler)
    logger.info("Bot iniciado. Esperando mensajes...")
    app.run_polling()

if __name__ == "__main__":
    main()
