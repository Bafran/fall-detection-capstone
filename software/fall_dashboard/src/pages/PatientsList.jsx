import { useState } from "react"
import { useNavigate } from "react-router-dom"
import Sidebar from "../components/layout/Sidebar.jsx"
import Topbar from "../components/layout/Topbar.jsx"
import { PATIENTS } from "../data/patients.js"
import { Users, ChevronRight } from "lucide-react"

export default function PatientsList() {
  const navigate = useNavigate()
  const [sidebarCollapsed, setSidebarCollapsed] = useState(false)

  return (
    <div
      style={{
        display: "grid",
        gridTemplateColumns: sidebarCollapsed ? "92px 1fr" : "260px 1fr",
        minHeight: "100vh",
        transition: "grid-template-columns 220ms ease",
      }}
    >
      <Sidebar
        collapsed={sidebarCollapsed}
        onToggle={() => setSidebarCollapsed((v) => !v)}
        currentPath="/patients"
      />
      <main style={styles.main}>
        <Topbar />
        <section style={styles.section}>
          <div style={styles.header}>
            <Users size={22} style={{ color: "var(--accent)" }} />
            <h1 style={styles.title}>Patients</h1>
            <p style={styles.subtitle}>Select a patient to view their dashboard</p>
          </div>
          <div style={styles.list}>
            {PATIENTS.map((patient) => (
              <div
                key={patient.id}
                role="button"
                tabIndex={0}
                onClick={() => navigate(`/patients/${patient.id}`)}
                onKeyDown={(e) =>
                  (e.key === "Enter" || e.key === " ") && navigate(`/patients/${patient.id}`)
                }
                style={styles.card}
              >
                <div style={styles.avatar}>{patient.name.charAt(0)}</div>
                <div style={styles.cardBody}>
                  <div style={styles.cardName}>{patient.name}</div>
                  <div style={styles.cardMeta}>View dashboard</div>
                </div>
                <ChevronRight size={18} style={styles.chevron} />
              </div>
            ))}
          </div>
        </section>
      </main>
    </div>
  )
}

const styles = {
  main: {
    padding: 18,
    display: "flex",
    flexDirection: "column",
    gap: 16,
  },
  section: {
    flex: 1,
    maxWidth: 560,
  },
  header: {
    marginBottom: 20,
  },
  title: {
    fontSize: 22,
    fontWeight: 800,
    margin: "8px 0 4px 0",
    color: "var(--text)",
  },
  subtitle: {
    fontSize: 14,
    color: "var(--muted)",
    margin: 0,
  },
  list: {
    display: "flex",
    flexDirection: "column",
    gap: 12,
  },
  card: {
    display: "flex",
    alignItems: "center",
    gap: 14,
    padding: "16px 18px",
    background: "var(--panel)",
    border: "1px solid var(--border)",
    borderRadius: "var(--radius)",
    boxShadow: "var(--shadow)",
    cursor: "pointer",
    transition: "background 0.15s ease, border-color 0.15s ease",
  },
  avatar: {
    width: 48,
    height: 48,
    borderRadius: 999,
    background: "rgba(14,165,164,.15)",
    display: "grid",
    placeItems: "center",
    fontWeight: 800,
    fontSize: 18,
    color: "var(--accent)",
    flexShrink: 0,
  },
  cardBody: { flex: 1, minWidth: 0 },
  cardName: { fontWeight: 700, fontSize: 16 },
  cardMeta: { fontSize: 12, color: "var(--muted)", marginTop: 2 },
  chevron: { color: "var(--muted)", flexShrink: 0 },
}
