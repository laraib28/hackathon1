import React from 'react';
import Layout from '@theme/Layout';
import { AuthProvider } from '../context/AuthContext';
import Login from '../components/Auth/Login';

export default function LoginPage() {
  return (
    <AuthProvider>
      <Layout title="Sign In" description="Sign in to access your learning dashboard">
        <Login />
      </Layout>
    </AuthProvider>
  );
}
