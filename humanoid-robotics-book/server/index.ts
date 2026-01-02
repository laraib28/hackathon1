/**
 * Express Server for Better Auth
 * Handles all authentication endpoints and chat API
 */

import 'dotenv/config';

import express from "express";
import cors from "cors";
import { pool } from "./db";
import { auth } from "./auth";
import { toNodeHandler } from "better-auth/node";
import personalizationRouter from "./personalization-api";

/* -------------------- APP SETUP -------------------- */

const app = express();
const PORT = process.env.PORT || 3001;

// CORS configuration with preflight handling
app.use(cors({
  origin: process.env.CLIENT_URL || "http://localhost:3000",
  credentials: true,
  methods: ['GET', 'POST', 'PUT', 'DELETE', 'OPTIONS'],
  allowedHeaders: ['Content-Type', 'Authorization'],
}));

// Handle preflight requests
app.options('*', cors());

app.use(express.json());

/* -------------------- HEALTH -------------------- */

app.get("/api/health", (req, res) => {
  res.json({
    status: "ok",
    message: "Better Auth server is running",
  });
});

/* -------------------- AUTH ROUTES -------------------- */

// Use Better Auth's Node handler for Express
app.all("/api/auth/*", toNodeHandler(auth));

/* -------------------- PERSONALIZATION ROUTES -------------------- */

// User profile, preferences, and skills management
app.use("/api/personalization", personalizationRouter);

/* -------------------- START SERVER (ASYNC SAFE) -------------------- */

async function startServer() {
  try {
    // ✅ Verify DB connection BEFORE server starts
    await pool.query("select 1");
    console.log("✅ PostgreSQL connected");

    app.listen(PORT, () => {
      console.log(`🚀 Better Auth server running on port ${PORT}`);
      console.log(`📍 Auth endpoint: http://localhost:${PORT}/api/auth`);
    });

  } catch (error) {
    console.error("❌ Failed to start server:", error);
    process.exit(1);
  }
}

startServer();

export default app;
