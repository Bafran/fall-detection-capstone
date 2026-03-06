export default function PatientPanel({ patient, safeZoneValue }) {
  const name = patient?.name ?? "Patient"
  const initial = name.charAt(0).toUpperCase()

  return (
    <section style={styles.panel}>
      <div style={styles.head}>
        <div style={{ fontWeight: 700 }}>Patient</div>
      </div>

      <div style={styles.body}>
        <div style={styles.row}>
          <div style={styles.avatar}>{initial}</div>
          <div>
            <div style={{ fontWeight: 800 }}>{name}</div>
            <div style={{ fontSize: 12, color: "var(--muted)" }}>Watch status: Connected</div>
          </div>
        </div>

        <div style={styles.kv}>
          <div>
            <div style={styles.k}>Last fall</div>
            <div style={styles.v}>None today</div>
          </div>
          <div>
            <div style={styles.k}>Safe zone</div>
            <div style={styles.v}>{safeZoneValue ?? "—"}</div>
          </div>
        </div>

        <div style={styles.actions}>
          <button style={styles.primary}>Emergency Call</button>
          <button style={styles.secondary}>Call Patient</button>
        </div>
      </div>
    </section>
  )
}

const styles = {
  panel: {
    background: "var(--panel)",
    border: "1px solid var(--border)",
    borderRadius: "var(--radius)",
    boxShadow: "var(--shadow)",
    overflow: "hidden",
    gridColumn: "2 / 3",
  },
  head: {
    padding: "12px 14px",
    borderBottom: "1px solid var(--border)",
  },
  body: { padding: 14, display: "flex", flexDirection: "column", gap: 12 },
  row: { display: "flex", alignItems: "center", gap: 10 },
  avatar: {
    width: 40, height: 40, borderRadius: 999,
    background: "rgba(14,165,164,.15)",
    display: "grid", placeItems: "center",
    fontWeight: 900, color: "var(--accent)",
  },
  kv: {
    display: "grid",
    gridTemplateColumns: "1fr 1fr",
    gap: 10,
    padding: 12,
    border: "1px solid var(--border)",
    borderRadius: 14,
  },
  k: { fontSize: 12, color: "var(--muted)" },
  v: { fontSize: 14, fontWeight: 700 },
  actions: { display: "grid", gridTemplateColumns: "1fr", gap: 8 },
  primary: {
    border: "none",
    background: "var(--danger)",
    color: "white",
    borderRadius: 14,
    padding: "10px 12px",
    fontWeight: 800,
    cursor: "pointer",
  },
  secondary: {
    border: "1px solid var(--border)",
    background: "white",
    borderRadius: 14,
    padding: "10px 12px",
    fontWeight: 700,
    cursor: "pointer",
  },
}