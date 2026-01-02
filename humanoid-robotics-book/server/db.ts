/**
 * Shared PostgreSQL Database Pool
 * Single pool instance used across the entire application
 */
import 'dotenv/config';
import { Pool } from 'pg';

// Remove channel_binding from connection string if present (causes issues with poolers)
const connectionString = process.env.DATABASE_URL?.replace('&channel_binding=require', '').replace('?channel_binding=require', '');

// Create a single PostgreSQL pool instance
// Neon and most cloud providers require SSL in all environments
export const pool = new Pool({
  connectionString: connectionString,
  ssl: {
    rejectUnauthorized: false
  },
  max: 5, // Lower pool size for Neon serverless
  min: 0, // Allow pool to scale to zero
  idleTimeoutMillis: 20000, // Close idle connections after 20s
  connectionTimeoutMillis: 30000, // Increased timeout for Neon
  allowExitOnIdle: true, // Allow process to exit when all connections idle
});

// Track if we've logged the initial connection
let hasLoggedConnection = false;

// Log connection status (only once to reduce noise)
pool.on('connect', () => {
  if (!hasLoggedConnection) {
    console.log('✅ PostgreSQL pool connected');
    hasLoggedConnection = true;
  }
});

pool.on('error', (err) => {
  console.error('❌ PostgreSQL pool error:', err);
});
