# Bench internet for drones (no eduroam on drones)

Laptop is wired to the TP-Link router (USB dongle, LAN IP **192.168.0.136**) while on
eduroam WiFi. The laptop NATs internet to the LAN; drones route through it over
**ethernet only** (drone WiFi config is never touched).

```
drone eth ──> TP-Link ──> laptop (.136, NAT) ──> eduroam ──> internet
```

## After a LAPTOP reboot

```bash
./bench_internet_laptop.sh     # re-adds the iptables NAT rules (needs sudo)
```

## After a DRONE reboot

Nothing — `bench-internet.service` runs `~/bench_internet.sh` at boot. It only acts
if the laptop answers at .136 (in the field it exits, leaving flight networking alone).
Manual rerun: `sudo ~/bench_internet.sh`. Installed on: **ghost** (others TODO).

It sets: default route via .136 (metric 50, ethernet), DNS 8.8.8.8/1.1.1.1 on the
ethernet interface, NTP on (Jetsons cold-boot to year 1969 — breaks HTTPS until synced).

## Gotchas

- Laptop LAN IP is a DHCP lease — if it stops being .136, update `GW=` in each drone's
  `~/bench_internet.sh` (better: reserve .136 for the dongle MAC in the TP-Link admin).
- Laptop dongle profile `tplink-lan` (nmcli) is never-default so eduroam keeps internet.
- Verify from a drone: `curl -sS -o /dev/null -w '%{http_code}\n' https://www.google.com` → 200.
