/**
 * Better Auth Server Configuration
 * FULL & FIXED VERSION
 */

import { betterAuth } from "better-auth";
import { Kysely, PostgresDialect } from "kysely";
import { pool } from "./db";

// ----------------------------
// Database (PostgreSQL)
// ----------------------------
const db = new Kysely({
  dialect: new PostgresDialect({
    pool: pool,
  }),
});

// ----------------------------
// Better Auth Config
// ----------------------------
export const auth = betterAuth({
  // ✅ Database - Kysely instance with type specification
  database: {
    db: db,
    type: "postgres",
  },

  // ✅ Email / Password Login
  emailAndPassword: {
    enabled: true,
    requireEmailVerification: false,
  },

  // ✅ Social Providers (optional)
  socialProviders: {
    google: {
      clientId: process.env.GOOGLE_CLIENT_ID || "",
      clientSecret: process.env.GOOGLE_CLIENT_SECRET || "",
      enabled: !!process.env.GOOGLE_CLIENT_ID,
    },
    github: {
      clientId: process.env.GITHUB_CLIENT_ID || "",
      clientSecret: process.env.GITHUB_CLIENT_SECRET || "",
      enabled: !!process.env.GITHUB_CLIENT_ID,
    },
  },

  // ✅ Session
  session: {
    expiresIn: 60 * 60 * 24 * 7, // 7 days
    updateAge: 60 * 60 * 24, // 1 day
  },

  // ✅ Security Secret
  secret:
    process.env.BETTER_AUTH_SECRET ||
    "dev-secret-change-this",

  // ✅ Auth Server URL
  baseURL:
    process.env.BETTER_AUTH_URL ||
    "http://localhost:3001",

  // ✅ 🔥 MAIN FIX (INVALID ORIGIN FIX)
  trustedOrigins: [
    "http://localhost:3000",
    "http://127.0.0.1:3000",
  ],

  // ✅ Advanced / Cookies
  advanced: {
    cookiePrefix: "better-auth",
    crossSubDomainCookies: {
      enabled: true,
    },
  },
});
