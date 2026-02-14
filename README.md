# 🌦️ LoRa APRS Weather Station Pro (IU1VDW)
**Standalone | Ultra-Low Power | Bosch BSEC2 High-Precision**

Benvenuti nel repository della **LoRa APRS Weather Station Pro**, un progetto sviluppato da **IU1VDW (Andrea)** per il monitoraggio meteorologico e ambientale in zone remote. 

A differenza di molti altri firmware, questo progetto è ottimizzato per il funzionamento **senza connessione internet** e **senza rete elettrica**, affidandosi esclusivamente alla trasmissione radio LoRa a 433 MHz e all'alimentazione da pannello solare.

---

## 💎 Caratteristica Esclusiva: Supporto Bosch BSEC2
Il punto di forza di questo codice è l'integrazione nativa della libreria **Bosch BSEC2** per il sensore **BME680**.

> [!IMPORTANT]
> **Oltre i dati grezzi:** Mentre la maggior parte delle stazioni legge solo temperatura e umidità, questo firmware processa i dati tramite gli algoritmi ufficiali Bosch per fornire un **Indice di Qualità dell'Aria (IAQ)** reale, compensato e calibrato automaticamente.

* **IAQ (Air Quality Index):** Monitoraggio professionale dell'aria.
* **CO2 equivalente & bVOC:** Analisi dei gas e dei composti organici volatili.
* **Calibrazione Persistente:** Gestione dello stato del sensore per mantenere la precisione anche dopo i cicli di sleep.

---

## 🔋 Efficienza Energetica (Solar Powered)
Il software è progettato per stazioni "mordi e fuggi" in alta quota o in zone isolate.

* **Light Sleep Mode:** L'ESP32 riduce i consumi al minimo tra una lettura e l'altra.
* **Duty-Cycle Intelligente:** Ottimizzato per batterie 18650 e piccoli pannelli fotovoltaici.

### Tabella dei Consumi (Stima)
| Fase Operativa | Consumo Medio | Durata |
| :--- | :--- | :--- |
| **Trasmissione RF LoRa** | ~120 mA | < 2 sec |
| **Campionamento BSEC2** | ~12 mA | ~3 sec |
| **Light Sleep** | ~0.8 - 1.5 mA | Configurabile (es. 10 min) |

---

## 📡 Architettura Standalone
Il sistema non richiede Wi-Fi. La comunicazione avviene interamente via radio.



```mermaid
graph TD
    A[Sensore BME680] -->|Dati Ambiente| B(Libreria BSEC2)
    B -->|IAQ + Meteo| C[ESP32 Core]
    C -->|Pacchetto APRS| D{Modulo LoRa 433MHz}
    D -.->|Onde Radio| E[iGate / Digipeater]
    E --> F[Mappa Mondiale APRS.fi]
