import logo from "../../assets/elbert_logo.png"
import "../../styles/globals.css"
import { Menu, LayoutGrid, Users, Shield, List, Settings } from "lucide-react"

export default function Sidebar({ collapsed, onToggle }) {
  return (
    <aside style={{ ...styles.sidebar, width: collapsed ? 84 : 260 }}>
      <div
        style={{
          ...styles.brandCard,
          justifyContent: collapsed ? "center" : "flex-start",
        }}
      >
        <button
          style={styles.menuBtn}
          onClick={onToggle}
          aria-label="Toggle sidebar"
        >
          <Menu size={18} />
        </button>

        {!collapsed && (
          <div style={styles.logoWrap}>
            <img src={logo} alt="ELBERT logo" style={styles.logo} />
          </div>
        )}
      </div>

      <nav style={styles.nav}>
        <SideItem collapsed={collapsed} active label="Dashboard" icon={<LayoutGrid size={18} />} />
        <SideItem collapsed={collapsed} label="Caregivers" icon={<Users size={18} />} />
        <SideItem collapsed={collapsed} label="Safe Zones" icon={<Shield size={18} />} />
        <SideItem collapsed={collapsed} label="Event Logs" icon={<List size={18} />} />
        <SideItem collapsed={collapsed} label="Settings" icon={<Settings size={18} />} />
      </nav>

      <div style={{ flex: 1 }} />

      {!collapsed ? (
        <div style={styles.hint}>Static mock for now. Later: ESP32 live data.</div>
      ) : null}
    </aside>
  )
}

function SideItem({ collapsed, label, icon, active }) {
  return (
    <a
      href="#"
      style={{
        ...styles.link,
        ...(active ? styles.active : null),
        justifyContent: collapsed ? "center" : "flex-start",
      }}
      title={collapsed ? label : undefined}
    >
      <span style={styles.iconSlot}>{icon}</span>
      {!collapsed ? <span>{label}</span> : null}
    </a>
  )
}

const styles = {
  sidebar: {
    background: "#0b2a4a",
    color: "#e5e7eb",
    padding: 14,
    display: "flex",
    flexDirection: "column",
    gap: 10,
    transition: "width 220ms ease",
    overflow: "hidden",
  },

  // Row: [hamburger] [logo block]
  brandCard: {
    padding: 12,
    borderRadius: 16,
    background: "rgba(255,255,255,.06)",
    display: "flex",
    alignItems: "center",
    gap: 14,
  },

  menuBtn: {
    width: 38,
    height: 38,
    borderRadius: 12,
    border: "none",
    background: "rgba(255,255,255,.00)",
    color: "#e5e7eb",
    display: "grid",
    placeItems: "center",
    cursor: "pointer",
    transition: "background 0.15s ease",
    paddingTop: 10
  },

  logo: {
    width: "85%",
    maxWidth: "200px",
    height: "auto",
    objectFit: "contain",
    marginLeft: 6,
  },

  brand: {
    padding: 12,
    borderRadius: 14,
    background: "rgba(255,255,255,.06)",
    display: "flex",
    justifyContent: "center",
    alignItems: "center",
    minHeight: 42,
  },

  nav: {
    display: "flex",
    flexDirection: "column",
    gap: 6,
    marginTop: 8,
  },

  link: {
    textDecoration: "none",
    padding: "10px 12px",
    borderRadius: 12,
    color: "#e5e7eb",
    fontSize: 13,
    display: "flex",
    alignItems: "center",
    gap: 10,
    background: "transparent",
  },

  iconSlot: {
    width: 22,
    display: "grid",
    placeItems: "center",
    color: "rgba(229,231,235,.92)",
    flexShrink: 0,
  },

  active: {
    background: "rgba(255,255,255,.08)",
  },

  hint: {
    fontSize: 12,
    opacity: 0.9,
    padding: 12,
    borderRadius: 14,
    background: "rgba(255,255,255,.06)",
  },
  logoWrap: {
    flex: 1,
    display: "flex",
    alignItems: "center",
  },
}