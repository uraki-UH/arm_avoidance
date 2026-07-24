import { Component, ReactNode } from 'react';

interface Props { children: ReactNode; }
interface State { failed: boolean; }

export class WebGLErrorBoundary extends Component<Props, State> {
    state: State = { failed: false };

    static getDerivedStateFromError(): State {
        return { failed: true };
    }

    // Do NOT auto-retry: repeated failures cause Chrome to disable WebGL entirely
    // (BindToCurrentSequence failed / GL_VENDOR = Disabled).
    // Only a browser restart can recover from that state.

    render() {
        if (this.state.failed) {
            return (
                <div style={{
                    display: 'flex', flexDirection: 'column', alignItems: 'center',
                    justifyContent: 'center', height: '100%', color: '#94a3b8',
                    background: '#07111f', gap: '16px', fontFamily: 'monospace', fontSize: '13px'
                }}>
                    <div style={{ color: '#f87171', fontSize: '15px' }}>WebGL が利用できません</div>
                    <div style={{ color: '#64748b', textAlign: 'center', lineHeight: 1.6 }}>
                        ブラウザがWebGLコンテキストを無効化しました。<br />
                        ブラウザを再起動してから再度アクセスしてください。
                    </div>
                    <div style={{ color: '#475569', fontSize: '11px' }}>
                        Chrome: すべてのタブを閉じて再起動
                    </div>
                </div>
            );
        }
        return this.props.children;
    }
}
