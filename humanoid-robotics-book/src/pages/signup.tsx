import React from 'react';
import Layout from '@theme/Layout';
import { AuthProvider } from '../context/AuthContext';
import Signup from '../components/Auth/Signup';

export default function SignupPage() {
  return (
    <AuthProvider>
      <Layout title="Create Account" description="Create an account to start your robotics learning journey">
        <Signup />
      </Layout>
    </AuthProvider>
  );
}
