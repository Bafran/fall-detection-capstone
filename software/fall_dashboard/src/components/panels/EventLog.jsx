const events = [
  { time: "11:45", msg: "Location refreshed", tone: "neutral" },
  { time: "11:32", msg: "Entered safe zone: Home", tone: "ok" },
  { time: "10:58", msg: "Device battery: 82%", tone: "neutral" },
]

export default function EventLog() {
  return (
    <section style={styles.panel}>
      <div style={styles.head}>
        <div style={{ fontWeight: 700 }}>Recent Events</div>
      </div>
      <div style={styles.body}>
        {events.map((e, i) => (
          <div key={i} style={styles.item}>
            <div style={styles.time}>{e.time}</div>
            <div style={styles.msg}>{e.msg}</div>
          </div>
        ))}
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
    gridColumn: "1 / 2",
  },
  head: {
    padding: "12px 14px",
    borderBottom: "1px solid var(--border)",
  },
  body: { padding: 14, display: "flex", flexDirection: "column", gap: 10 },
  item: {
    display: "grid",
    gridTemplateColumns: "60px 1fr",
    gap: 10,
    padding: 10,
    border: "1px solid var(--border)",
    borderRadius: 14,
  },
  time: { fontSize: 12, color: "var(--muted)", fontWeight: 700 },
  msg: { fontSize: 13 },
}