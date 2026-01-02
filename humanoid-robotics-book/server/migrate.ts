/**
 * Database Migration Script
 * Creates necessary tables for Better Auth in PostgreSQL
 */
import { config } from "dotenv";
import { Pool } from "pg";

// Load environment variables from .env file
config();

// Verify DATABASE_URL is set
if (!process.env.DATABASE_URL) {
  console.error("❌ DATABASE_URL not found in environment variables");
  console.error("Please check your .env file");
  process.exit(1);
}

console.log("🔗 Connecting to database...");

const pool = new Pool({
  connectionString: process.env.DATABASE_URL,
  ssl: {
    rejectUnauthorized: false
  },
});

const createTables = `
-- Users table (Better Auth v1.4.8 format - singular name with quoted columns)
CREATE TABLE IF NOT EXISTS "user" (
  id TEXT PRIMARY KEY,
  email TEXT NOT NULL UNIQUE,
  "emailVerified" BOOLEAN DEFAULT FALSE,
  name TEXT,
  image TEXT,
  "createdAt" TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  "updatedAt" TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Accounts table (Better Auth v1.4.8 format)
CREATE TABLE IF NOT EXISTS "account" (
  id TEXT PRIMARY KEY,
  "accountId" TEXT NOT NULL,
  "providerId" TEXT NOT NULL,
  "userId" TEXT NOT NULL REFERENCES "user"(id) ON DELETE CASCADE,
  "accessToken" TEXT,
  "refreshToken" TEXT,
  "expiresAt" BIGINT,
  scope TEXT,
  password TEXT,
  "createdAt" TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  "updatedAt" TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  UNIQUE("providerId", "accountId")
);

-- Sessions table (Better Auth v1.4.8 format)
CREATE TABLE IF NOT EXISTS "session" (
  id TEXT PRIMARY KEY,
  "userId" TEXT NOT NULL REFERENCES "user"(id) ON DELETE CASCADE,
  "expiresAt" TIMESTAMP NOT NULL,
  token TEXT NOT NULL UNIQUE,
  "ipAddress" TEXT,
  "userAgent" TEXT,
  "createdAt" TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  "updatedAt" TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Verification table (Better Auth v1.4.8 format)
CREATE TABLE IF NOT EXISTS "verification" (
  id TEXT PRIMARY KEY,
  identifier TEXT NOT NULL,
  value TEXT NOT NULL UNIQUE,
  "expiresAt" TIMESTAMP NOT NULL,
  "createdAt" TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
  "updatedAt" TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Create indexes for better performance
CREATE INDEX IF NOT EXISTS idx_account_user_id ON "account"("userId");
CREATE INDEX IF NOT EXISTS idx_session_user_id ON "session"("userId");
CREATE INDEX IF NOT EXISTS idx_session_token ON "session"(token);
`;

export async function runMigration() {
  try {
    console.log("🔄 Running database migrations...");

    await pool.query(createTables);

    console.log("✅ Database migrations completed successfully!");
    console.log("📊 Tables created:");
    console.log("   - user");
    console.log("   - account");
    console.log("   - session");
    console.log("   - verification");

    await pool.end();
  } catch (error) {
    console.error("❌ Migration failed:", error);
    process.exit(1);
  }
}

// Run migration if this file is executed directly
if (require.main === module) {
  runMigration();
}
