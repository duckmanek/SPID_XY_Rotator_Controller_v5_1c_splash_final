# 🛰️ SPID XY ROTATOR CONTROLLER by SP1JMF
**Wersja:** v5.1c SPLASH FINAL

Zaawansowana aplikacja do sterowania rotorami anten SPID MD-03 i kompatybilnymi, z funkcjami śledzenia satelitów, integracją z PSTrotator, oraz obsługą przez USB i TCP.

---

## 🌟 Funkcje

- 🔁 **Sterowanie SPID MD-03 (ROT2PROG)** – w trybie USB lub TCP (port 23)
- 🌍 **Śledzenie satelitów (LEO, QO-100, TEVEL)** – automatyczny import TLE
- 💡 **Autoaktualizacja plików TLE** (AMSAT + Celestrak)
- 🛰️ **Interfejs graficzny (Tkinter GUI)** z nowoczesnym layoutem
- 🎛️ **Tryb binarny / impulsowy 0.1°** dla pełnej precyzji
- ⚙️ **Komunikacja z PSTrotator** (TCP port 12000, UDP forward 12001)
- 🔇 **Przycisk ON/OFF PSTrotator** (zielony / czerwony)
- 💾 **Zapis ustawień** w `config.ini` (QTH, porty, geometria GUI)
- 💠 **Ikona SPID XY + ekran powitalny (splash)** w stylu satelitarnym
- 📈 **Wizualizacja przelotu satelity** z wartościami Az/El w stopniach
- 🚀 **Zgodność z Windows 10/11** i Python 3.10–3.12

---

## 🧩 Wymagane biblioteki

Zainstaluj zależności:
```bash
pip install pyserial pyorbital requests pillow matplotlib pyinstaller
